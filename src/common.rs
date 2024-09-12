use core::{borrow::Borrow, fmt::Pointer, ops::Deref, pin::Pin};

use crate::IRQPort;




pub trait PortDriver {
    type Error;

    /// Set all pins in `mask_high` to HIGH and all pins in `mask_low` to LOW.
    ///
    /// The driver should implements this such that all pins change state at the same time.
    async fn set(&mut self, mask_high: u32, mask_low: u32) -> Result<(), Self::Error>;

    /// Check whether pins in `mask_high` were set HIGH and pins in `mask_low` were set LOW.
    ///
    /// For each pin in either of the masks, the returned `u32` should have a 1 if they meet the
    /// expected state and a 0 otherwise.  All other bits MUST always stay 0.
    ///
    /// If a bit is set in both `mask_high` and `mask_low`, the resulting bit must be 1.
    fn is_set(&mut self, mask_high: u32, mask_low: u32) -> Result<u32, Self::Error>;

    /// Check whether pins in `mask_high` are driven HIGH and pins in `mask_low` are driven LOW.
    ///
    /// For each pin in either of the masks, the returned `u32` should have a 1 if they meet the
    /// expected state and a 0 otherwise.  All other bits MUST always stay 0.
    ///
    /// If a bit is set in both `mask_high` and `mask_low`, the resulting bit must be 1.
    async fn get(&mut self, mask_high: u32, mask_low: u32) -> Result<u32, Self::Error>;

    async fn toggle(&mut self, mask: u32) -> Result<(), Self::Error> {
        // for all pins which are currently low, make them high.
        let mask_high = self.is_set(0, mask)?;
        // for all pins which are currently high, make them low.
        let mask_low = self.is_set(mask, 0)?;
        self.set(mask_high, mask_low).await
    }
}

pub trait PortDriverDirection: PortDriver {
    /// Set the direction for all pins in `mask` to direction `dir`.
    ///
    /// To prevent electrical glitches, when making pins outputs, the `state` can be either `true`
    /// or `false` to immediately put the pin HIGH or LOW upon switching.
    async fn set_direction(&mut self, mask: u32, dir: Direction, state: bool) -> Result<(), Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Input,
    Output,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Polarity {
    ActiveLow,
    ActiveHigh,
}

pub trait PortDriverPolarity: PortDriver {
    /// Set the polarity of all pins in `mask` either `inverted` or not.
    async fn set_polarity(&mut self, mask: u32, polarity: Polarity) -> Result<(), Self::Error>;
}

pub trait PortDriverPullDown: PortDriver {
    /// Enable pull-downs for pins in mask or set the pin to floating if enable is false.
    async fn set_pull_down(&mut self, mask: u32, enable: bool) -> Result<(), Self::Error>;
}

pub trait PortDriverPullUp: PortDriver {
    /// Enable pull-ups for pins in mask or set the pin to floating if enable is false.
    async fn set_pull_up(&mut self, mask: u32, enable: bool) -> Result<(), Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Drive {
    PushPull,
    OpenDrain,
}

pub trait PortDriverOutputDrive: PortDriver {
    /// Switches the drive of the pins in `mask` to either push-pull or open-drain.
    async fn set_drive(&mut self, mask: u32, polarity: Drive) -> Result<(), Self::Error>;
}

/// Interrupt types for port expander pins.
/// Not all types are supported by all port expanders.
/// The driver should implement the types it supports and can either return an error for the types it does not support, or implement them in software.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptType {
    /// Interrupt triggered on a falling edge.
    Falling = 1,
    /// Interrupt triggered on a rising edge.
    Rising,
    /// Interrupt triggered on any edge.
    Both,
    /// Interrupt triggered when the pin is high.
    /// Behaviour can be one-shot or latched.
    High,
    /// Interrupt triggered when the pin is low.
    /// Behaviour can be one-shot or latched.
    Low,
    Err,
}

pub trait PortDriverInterrupts: PortDriver {

    async fn fetch_interrupt_state(&mut self) -> Result<(), Self::Error>;

}

pub trait PortDriverIrqMask: PortDriver {
    /// Set/clear the interrupt mask of the port expander.
    async fn configure_interrupts(&mut self, mask_set: u32, mask_clear: u32, interrupt: InterruptType) -> Result<(), Self::Error>;
}


pub trait PortDriverExtI<ISR, ISRRC>: PortDriverISR<ISR, ISRRC> where ISRRC: Deref<Target = ISR>
{
    async fn configure_int_pin(&mut self, mask: u32, polarity: Polarity, drive: Drive) -> Result<(), Self::Error>;

}


/// This trait provides a reference to the interrupt service routine (ISR) logic for the port expander.
/// RC is generic over smart pointer types like Rc and Arc. Heapless equivalents should also work.
pub trait PortDriverISR<ISR, ISRRC>: PortDriverInterrupts + PortDriverIrqMask where ISRRC: Deref<Target = ISR>
{
    fn get_isr(&mut self) -> ISRRC;
}

/// Pin Modes
pub mod mode {
    /// Trait for pin-modes which can be used to set a logic level.
    pub trait HasOutput {}
    /// Trait for pin-modes which can be used to read a logic level.
    pub trait HasInput {}
    /// Trait for pins which can be used to trigger an interrupt.
    pub trait HasInterrupt {}
    /// Trait for pins which can be used as open-drain outputs.
    pub trait HasOpenDrain {}

    /// Pin configured as an input.
    pub struct Input;
    impl HasInput for Input {}

    /// Pin configured as an input with interrupt support.
    pub struct ISRInput;
    impl HasInput for ISRInput {}
    impl HasInterrupt for ISRInput {}

    /// Pin configured as an output.
    pub struct Output;
    impl HasOutput for Output {}

    /// Pin configured as a quasi-bidirectional input/output.
    pub struct QuasiBidirectional;
    impl HasInput for QuasiBidirectional {}
    impl HasOutput for QuasiBidirectional {}

    /// Pin configured as a quasi-bidirectional input/output with interrupt support.
    pub struct ISRQuasiBidirectional;
    impl HasInput for ISRQuasiBidirectional {}
    impl HasOutput for ISRQuasiBidirectional {}
    impl HasInterrupt for ISRQuasiBidirectional {}

    /// Trait that describes a pin on the microcontroller that can be used as an external interrupt output.
    pub trait HasExtI {}

    /// Pin configured as an external interrupt output.
    /// Unused currently, but may be used in the future.
    pub struct ExtI;
    impl HasExtI for ExtI {}

}

pub trait RefPtr: Clone + Deref {
    fn new(value: Self::Target) -> Self;
    fn make_mut(self_: &mut Self) -> &mut Self::Target
    where
        Self::Target: Clone;

    fn try_unwrap(this: Self) -> Result<<Self as Deref>::Target, Self> where <Self as Deref>::Target: Sized;

    fn into_inner(this: Self) -> Option<<Self as Deref>::Target> where <Self as Deref>::Target: Sized;
}

impl<T> RefPtr for alloc::rc::Rc<T> {
    fn new(value: Self::Target) -> Self {
        alloc::rc::Rc::new(value)
    }
    fn make_mut(self_: &mut Self) -> &mut Self::Target
    where
        T: Clone,
    {
        alloc::rc::Rc::make_mut(self_)
    }

    fn try_unwrap(this: Self) -> Result<<Self as Deref>::Target, Self> {
        alloc::rc::Rc::try_unwrap(this)
    }

    fn into_inner(this: Self) -> Option<<Self as Deref>::Target> {
        alloc::rc::Rc::into_inner(this)
    }
}

#[cfg(feature = "std")]
impl<T> RefPtr for std::sync::Arc<T> {
    fn new(value: Self::Target) -> Self {
        std::sync::Arc::new(value)
    }
    fn make_mut(self_: &mut Self) -> &mut Self::Target
    where
        T: Clone,
    {
        std::sync::Arc::make_mut(self_)
    }

    fn try_unwrap(this: Self) -> Result<<Self as Deref>::Target, Self> {
        std::sync::Arc::try_unwrap(this)
    }

    fn into_inner(this: Self) -> Option<<Self as Deref>::Target> {
        std::sync::Arc::into_inner(this)
    }
}

impl<T> RefPtr for rclite::Rc<T> {
    fn new(value: Self::Target) -> Self {
        rclite::Rc::new(value)
    }
    fn make_mut(self_: &mut Self) -> &mut Self::Target
    where
        T: Clone,
    {
        rclite::Rc::make_mut(self_)
    }

    fn try_unwrap(this: Self) -> Result<<Self as Deref>::Target, Self> {
        rclite::Rc::try_unwrap(this)
    }

    fn into_inner(this: Self) -> Option<<Self as Deref>::Target> {
        rclite::Rc::into_inner(this)
    }
}

impl<T> RefPtr for rclite::Arc<T> {
    fn new(value: Self::Target) -> Self {
        rclite::Arc::new(value)
    }
    fn make_mut(self_: &mut Self) -> &mut Self::Target
    where
        T: Clone,
    {
        rclite::Arc::make_mut(self_)
    }

    fn try_unwrap(this: Self) -> Result<<Self as Deref>::Target, Self> {
        rclite::Arc::try_unwrap(this)
    }

    fn into_inner(this: Self) -> Option<<Self as Deref>::Target> {
        rclite::Arc::into_inner(this)
    }
}
pub trait Shared<T> {
    type Ptr: RefPtr<Target = T>;

    fn new(value: <Self::Ptr as Deref>::Target) -> Self::Ptr {
        Self::Ptr::new(value)
    }

    fn make_mut(this: &mut Self::Ptr) -> &mut <Self::Ptr as Deref>::Target
    where
        <Self::Ptr as Deref>::Target: Clone,
    {
        Self::Ptr::make_mut(this)
    }

    fn try_unwrap(this: Self::Ptr) -> Result<<Self::Ptr as Deref>::Target, Self::Ptr> {
        Self::Ptr::try_unwrap(this)
    }

    fn into_inner(this: Self::Ptr) -> Option<<Self::Ptr as Deref>::Target> {
        Self::Ptr::into_inner(this)
    }   
}

impl<T> Shared<T> for alloc::rc::Rc<()> {
    type Ptr = alloc::rc::Rc<T>;
}

impl<T> Shared<T> for rclite::Rc<()> {
    type Ptr = rclite::Rc<T>;
}

impl<T> Shared<T> for rclite::Arc<()> {
    type Ptr = rclite::Arc<T>;
}

#[cfg(feature = "std")]
impl<T> Shared<T> for std::sync::Arc<()> {
    type Ptr = Arc<T>;
}


