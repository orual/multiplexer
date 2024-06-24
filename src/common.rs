use core::{borrow::Borrow, ops::Deref};

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


#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptType {
    Falling = 1,
    Rising,
    Both,
    High,
    Low,
}

pub trait PortDriverInterrupts: PortDriver {

    async fn fetch_interrupt_state(&mut self) -> Result<(), Self::Error>;

}

pub trait PortDriverIrqMask: PortDriver {
    /// Set/clear the interrupt mask of the port expander.
    async fn configure_interrupts(&mut self, mask_set: u32, mask_clear: u32, interrupt: InterruptType) -> Result<(), Self::Error>;
}


pub trait PortDriverExtI<ISR,RC>: PortDriverISR<ISR, RC> 
where RC: Deref<Target = ISR> + Clone + AsRef<ISR> + Borrow<ISR>
{
    async fn configure_int_pin(&mut self, mask: u32, polarity: Polarity, drive: Drive) -> Result<(), Self::Error>;

}


/// This trait provides a reference to the interrupt service routine (ISR) logic for the port expander.
/// RC is generic over smart pointer types like Rc and Arc. Heapless equivalents should also work.
pub trait PortDriverISR<ISR, RC>: PortDriverInterrupts + PortDriverIrqMask 
where RC: Deref<Target = ISR> + Clone + AsRef<ISR> + Borrow<ISR>
{
    fn get_isr(&mut self) -> RC;
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

    pub struct ISRQuasiBidirectional;
    impl HasInput for ISRQuasiBidirectional {}
    impl HasOutput for ISRQuasiBidirectional {}
    impl HasInterrupt for ISRQuasiBidirectional {}

    /// Trait that describes a pin on the microcontroller that can be used as an external interrupt output.
    pub trait HasExtI {}

    pub struct ExtI;
    impl HasExtI for ExtI {}

}
