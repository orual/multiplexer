use core::cell::RefCell;
use core::future::Future;
use core::task::{Context, Poll, Waker};

use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex};
use embassy_sync::blocking_mutex::Mutex;
use embedded_hal_async::digital::Wait;
use embassy_sync::mutex::Mutex as AsyncMutex;

use crate::common::{RefPtr, Shared};

/// Interrupt request (IRQ) structure
/// Used to represent the active interrupt for a single pin and to wake its associated task
#[derive(Debug, Clone)]
pub struct Irq {
    pub pin_mask: u32,
    pub interrupt: crate::InterruptType,
    pub state: bool,
    pub waker: Option<Waker>,
}

/// Interrupt request (IRQ) port
/// Software structure for managing interrupts.
/// Functions similarly to the `embassy_sync::waitqueue::MultiWakerRegistration`
/// However, it selectively wakes tasks based on the pin mask and interrupt type.
pub struct IrqPort<const N: usize>{
    irqs: Mutex<CriticalSectionRawMutex, RefCell<heapless::Vec<Irq, N>>>,
}

impl<const N: usize> IrqPort<N> {
    /// Create a new IRQ port
    pub const fn new() -> Self {
        Self { irqs: Mutex::new(RefCell::new(heapless::Vec::new())) }
    }
}

pub enum IRQError {
    InvalidPinMask,
    InvalidInterruptType,
    IRQNotFound
}

impl<const N: usize> IRQPort for IrqPort<N> {
    fn register_irq(&self, pin_mask: u32, interrupt: crate::InterruptType) -> Irq {
        let irq = Irq {
            pin_mask,
            interrupt,
            state: false,
            waker: None
        };
        self.irqs.lock(|irqs| {
            let mut irqs = irqs.borrow_mut();
            if let Some(irq_curr) = irqs.iter_mut().find(|irq| irq.pin_mask == pin_mask) {
                irq_curr.interrupt = interrupt;
                irq_curr.state = false;
                irq_curr.clone()
            } else {
                irqs.push(irq.clone()).unwrap();
                irq
            }
        })
        
    }

    fn is_registered(&self, pin_mask: u32, interrupt: crate::InterruptType) -> bool {
        self.irqs.lock(|irqs| {
            let irqs = irqs.borrow();
            irqs.iter().any(|irq| irq.pin_mask == pin_mask && irq.interrupt == interrupt)
        })
    }

    fn unregister_irq(&self, pin_mask: u32) {
        self.irqs.lock(|irqs| {
            let mut irqs = irqs.borrow_mut();
            irqs.retain(|irq| irq.pin_mask != pin_mask);
        });
    }

    fn get_irq<'s,'a>(&'s self, pin_mask: u32) -> Option<IRQFuture<'s, 'a, Self>> where Self: Sized {
        if self.irqs.lock(|irqs| {
            let irqs = irqs.borrow_mut();
            let irq = irqs.iter().find(|&irq| irq.pin_mask == pin_mask);
            irq.map(|irq| irq.clone())
        }).is_some() {
            Some(IRQFuture {
                pin_mask,
                isr: self,
                _a: core::marker::PhantomData
            })
        } else {
            None
        }
        
    }

    fn push_irq(&self, changes: (u32, u32)) {
        self.irqs.lock(|irqs| {
            let mut irqs = irqs.borrow_mut();
            for irq in irqs.iter_mut() {
                if let Some(waker) = &irq.waker {
                    match irq.interrupt {
                        crate::InterruptType::Falling 
                        if changes.0 & irq.pin_mask != 0 && changes.1 & irq.pin_mask == 0 => {
                            irq.state = true;
                            waker.wake_by_ref();
                        },
                        crate::InterruptType::Rising 
                        if changes.0 & irq.pin_mask == 0 && changes.1 & irq.pin_mask != 0 => {
                            irq.state = true;
                            waker.wake_by_ref();
                        },
                        crate::InterruptType::Both 
                        if changes.0 & irq.pin_mask != changes.1 & irq.pin_mask => {
                            irq.state = true;
                            waker.wake_by_ref();
                        },
                        crate::InterruptType::High 
                        if changes.1 & irq.pin_mask != 0 => {
                            irq.state = true;
                            waker.wake_by_ref();
                        },
                        crate::InterruptType::Low 
                        if changes.1 & irq.pin_mask == 0 => {
                            irq.state = true;
                            waker.wake_by_ref();
                        },
                        _ => irq.state = false,
                    }
                }
            }
        })
    }

    fn poll_irq(&self, pin_mask: u32, cx: &mut Context<'_>) -> Poll<Result<(), IRQError>> {
        self.irqs.lock(|irqs| {
            let mut irqs = irqs.borrow_mut();
            let irq = irqs.iter_mut().find(|irq| irq.pin_mask == pin_mask);
            if let Some(irq) = irq {
                if irq.state {
                    irq.state = false;
                    Poll::Ready(Ok(()))
                } else {
                    if let Some(waker) = &irq.waker {
                        if waker.will_wake(cx.waker()) {
                            return Poll::Pending;
                        }
                    }
                    irq.waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            } else {
                Poll::Ready(Err(IRQError::IRQNotFound))
            }
        })
        
    }
}

/// Interrupt request (IRQ) port trait
/// This is the pin/client side of the IRQ system.
pub trait IRQPort {
    /// Register a new interrupt for a pin
    fn register_irq(&self, pin_mask: u32, interrupt: crate::InterruptType) -> Irq;
    /// Check if an interrupt is registered for a pin
    fn is_registered(&self, pin_mask: u32, interrupt: crate::InterruptType) -> bool;
    /// Get the corresponding interrupt future for a pin, to `await` on.
    fn get_irq<'s,'a>(&'s self, pin_mask: u32) -> Option<IRQFuture<'s, 'a, Self>> where Self: Sized;
    /// Unregister an interrupt for a pin
    fn unregister_irq(&self, pin_mask: u32);
    /// Push a new interrupt to the IRQ port, for distribution to the associated tasks
    fn push_irq(&self, changes: (u32, u32));
    /// Poll the IRQ port for an interrupt for a specific pin or pins
    /// Used internally by `IRQFuture` to allow `await`ing for an interrupt
    fn poll_irq(&self, pin_mask: u32, cx: &mut Context<'_>) -> Poll<Result<(), IRQError>>;
}

impl<RC, T: IRQPort> IRQPort for RC
where
RC: core::ops::Deref<Target = T> + Clone {
    fn register_irq(&self, pin_mask: u32, interrupt: crate::InterruptType) -> Irq {
        self.deref().register_irq(pin_mask, interrupt)
    }

    fn is_registered(&self, pin_mask: u32, interrupt: crate::InterruptType) -> bool {
        self.deref().is_registered(pin_mask, interrupt)
    }

    fn get_irq<'s,'a>(&'s self, pin_mask: u32) -> Option<IRQFuture<'s, 'a, Self>> where Self: Sized {
        self.deref().get_irq(pin_mask).map(|_| IRQFuture {
            pin_mask,
            isr: self,
            _a: core::marker::PhantomData
        })
    }

    fn unregister_irq(&self, pin_mask: u32) {
        self.deref().unregister_irq(pin_mask)
    }

    fn push_irq(&self, changes: (u32, u32)) {
        self.deref().push_irq(changes)
    }

    fn poll_irq(&self, pin_mask: u32, cx: &mut Context<'_>) -> Poll<Result<(), IRQError>> {
        self.deref().poll_irq(pin_mask, cx)
    }
}



pub trait ISRPort {
    
    async fn wait_for_interrupt(&self) -> ();
}

/// External interrupt pin (MCU-side input)
/// Pin must implement the `Wait` trait from `embedded-hal-async`
pub struct ExtIPin<W>(pub W);

impl<RM, W, E> ISRPort for ExtIPin<AsyncMutex<RM, W>> 
where 
RM: RawMutex,
E: core::fmt::Debug,
W: Wait<Error = E> 
{
    async fn wait_for_interrupt(&self) {
        self.0.lock().await.wait_for_any_edge().await.unwrap();
    }
}

impl<RC, T: ISRPort> ISRPort for RC
where
RC: core::ops::Deref<Target = T> + ?Sized
{
    async fn wait_for_interrupt(&self) { self.deref().wait_for_interrupt().await }
}

/// Future for a pin interrupt request.
pub struct IRQFuture<'s, 'a, ISR: IRQPort> {
    pin_mask: u32,
    isr: &'s ISR,
    _a: core::marker::PhantomData<&'a ()>
}

impl<'s, 'a, ISR: IRQPort> Future for IRQFuture<'s, 'a, ISR> {
    type Output = ();

    fn poll(self: core::pin::Pin<&mut IRQFuture<'s, 'a, ISR>>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        IRQPort::poll_irq(self.isr, self.pin_mask, cx).map(|_| ())
    }
}