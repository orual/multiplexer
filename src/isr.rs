use core::cell::RefCell;
use core::future::Future;
use core::task::{Context, Poll, Waker};

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embedded_hal_async::digital::Wait;
use embassy_sync::mutex::Mutex as AsyncMutex;

#[derive(Debug, Clone)]
pub struct Irq {
    pub pin_mask: u32,
    pub interrupt: crate::InterruptType,
    pub state: bool,
    pub waker: Option<Waker>,
}

pub struct IrqPort<M: RawMutex, const N: usize>{
    irqs: Mutex<M, RefCell<heapless::Vec<Irq, N>>>,
}

impl<M: RawMutex, const N: usize> IrqPort<M, N> {
    pub const fn new() -> Self {
        Self { irqs: Mutex::const_new(M::INIT, RefCell::new(heapless::Vec::new())) }
    }

    pub fn push_irq(&self, pin_mask: u32) -> Result<(), IRQError>{
        self.irqs.lock(|irqs| {
            let mut irqs = irqs.borrow_mut();
            if let Some(irq) = irqs.iter_mut().find(|irq| irq.pin_mask == pin_mask) {
                irq.state = true;
                if let Some(waker) = &irq.waker {
                    waker.wake_by_ref();
                    Ok(())
                } else {
                    Err(IRQError::IRQNotFound)
                }
            } else {
                Err(IRQError::InvalidPinMask)
            }
        })
    }
}

pub enum IRQError {
    InvalidPinMask,
    InvalidInterruptType,
    IRQNotFound
}

impl<M: RawMutex, const N: usize> IRQPort for IrqPort<M, N> {
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

pub trait IRQPort {
    fn register_irq(&self, pin_mask: u32, interrupt: crate::InterruptType) -> Irq;
    fn is_registered(&self, pin_mask: u32, interrupt: crate::InterruptType) -> bool;
    fn get_irq<'s,'a>(&'s self, pin_mask: u32) -> Option<IRQFuture<'s, 'a, Self>> where Self: Sized;
    fn unregister_irq(&self, pin_mask: u32);
    fn push_irq(&self, changes: (u32, u32));
    fn poll_irq(&self, pin_mask: u32, cx: &mut Context<'_>) -> Poll<Result<(), IRQError>>;
}

impl<RC, T: IRQPort> IRQPort for RC
where
RC: core::ops::Deref<Target = T> + ?Sized
{
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