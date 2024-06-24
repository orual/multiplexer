
use core::marker::PhantomData;
use embassy_futures::block_on;
use embassy_sync::{blocking_mutex::raw::RawMutex, mutex::{Mutex, MutexGuard}};
use embedded_hal::digital::{self as hal_digital, ErrorType};
use embedded_hal_async::digital::Wait;




/// Representation of a port-expander pin.
///
/// `Pin` is not constructed directly, this type is created by instanciating a port-expander and
/// then getting access to all its pins using the `.split()` method.
pub struct Pin<'a, MODE, PD, RM: RawMutex, IRQ, RC> {
    pin_mask: u32,
    port_driver: &'a Mutex<RM, PD>,
    irq: Option<RC>,
    _m: PhantomData<MODE>,
    _g: PhantomData<RM>,
    _i: PhantomData<IRQ>,
}

impl<'a, MODE, RM, PD, IRQ, RC> Pin<'a, MODE, PD, RM, IRQ, RC>
where
    PD: crate::PortDriver,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    RC: core::ops::Deref<Target = IRQ> + Clone + AsRef<IRQ> + core::borrow::Borrow<IRQ> 
{
    pub(crate) fn new(pin_number: u8, port_driver: &'a Mutex<RM, PD>) -> Self {
        assert!(pin_number < 32);
        Self {
            pin_mask: 1 << pin_number,
            port_driver,
            irq: None,
            _m: PhantomData,
            _g: PhantomData,
            _i: PhantomData,
        }
    }

    pub fn pin_mask(&self) -> u32 {
        self.pin_mask
    }

    pub(crate) async fn port_driver(&self) -> MutexGuard<RM, PD> {
        self.port_driver.lock().await
    }

    pub fn access_port_driver<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut PD) -> R,
    {
        self.port_driver.try_lock().map(|mut guard| {
            let pd: &mut PD = &mut *guard;
            f(&mut *pd)
        }).unwrap()
    }

}



/// Error type for [`Pin`] which implements [`embedded_hal::digital::Error`].
#[derive(Debug)]
pub struct PinError<PDE> {
    driver_error: PDE,
}

impl<PDE> PinError<PDE> {
    /// The upstream port driver error that occurred
    pub fn driver_error(&self) -> &PDE {
        &self.driver_error
    }
}

impl<PDE> hal_digital::Error for PinError<PDE>
where
    PDE: core::fmt::Debug,
{
    fn kind(&self) -> hal_digital::ErrorKind {
        hal_digital::ErrorKind::Other
    }
}

impl<PDE> From<PDE> for PinError<PDE> {
    fn from(value: PDE) -> Self {
        Self {
            driver_error: value,
        }
    }
}

impl<'a, MODE, RM, PD, ISR, RC> ErrorType for Pin<'a, MODE, PD, RM, ISR, RC>
where
    PD: crate::PortDriver + crate::PortDriverDirection,
    RM: RawMutex,
    PD::Error: core::fmt::Debug,
    ISR: crate::IRQPort,
{
    type Error = PinError<PD::Error>;
}

impl<'a, MODE, RM, PD, IRQ, RC> Pin<'a, MODE, PD, RM, IRQ, RC>
where
    PD: crate::PortDriver + crate::PortDriverDirection,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    RC: core::ops::Deref<Target = IRQ> + Clone + AsRef<IRQ> + core::borrow::Borrow<IRQ> 
{
    /// Configure this pin as an input.
    ///
    /// The exact electrical details depend on the port-expander device which is used.
    pub async fn into_input(self) -> Result<Pin<'a, crate::mode::Input, PD, RM, IRQ, RC>, PinError<PD::Error>> {
        self.port_driver().await.set_direction(self.pin_mask, crate::Direction::Input, false).await?;
        Ok(Pin {
            pin_mask: self.pin_mask,
            port_driver: self.port_driver,
            irq: self.irq,
            _m: PhantomData,
            _g: PhantomData,
            _i: PhantomData,
        })
    }

    /// Configure this pin as an output with an initial LOW state.
    ///
    /// The LOW state is, as long as he port-expander chip allows this, entered without any
    /// electrical glitch.
    pub async fn into_output(self) -> Result<Pin<'a, crate::mode::Output, PD, RM, IRQ, RC>, PinError<PD::Error>> {
        self.port_driver().await.set_direction(self.pin_mask, crate::Direction::Output, false).await?;
        Ok(Pin {
            pin_mask: self.pin_mask,
            port_driver: self.port_driver,
            irq: self.irq,
            _m: PhantomData,
            _g: PhantomData,
            _i: PhantomData,
        })
    }

    /// Configure this pin as an output with an initial HIGH state.
    ///
    /// The HIGH state is, as long as he port-expander chip allows this, entered without any
    /// electrical glitch.
    pub async fn into_output_high(
        self,
    ) -> Result<Pin<'a, crate::mode::Output, PD, RM, IRQ, RC>, PinError<PD::Error>> {
        self.port_driver().await.set_direction(self.pin_mask, crate::Direction::Output, true).await?;
        Ok(Pin {
            pin_mask: self.pin_mask,
            port_driver: self.port_driver,
            irq: self.irq,
            _m: PhantomData,
            _g: PhantomData,
            _i: PhantomData,
        })
    }
}

impl <'a, MODE: crate::mode::HasInterrupt, RM, PD, IRQ, RC> Pin<'a, MODE,  PD, RM, IRQ, RC> 
where 
    PD: crate::PortDriver + crate::PortDriverISR<IRQ, RC>,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    RC: core::ops::Deref<Target = IRQ> + Clone + AsRef<IRQ> + core::borrow::Borrow<IRQ> 
{
    pub async fn into_isr_pin(self) -> Result<Pin<'a, crate::mode::ISRInput, PD, RM, IRQ, RC>, PinError<PD::Error>> {
        if self.irq.is_some() { 
            Ok(Pin {
                pin_mask: self.pin_mask,
                port_driver: self.port_driver,
                irq: self.irq,
                _m: PhantomData,
                _g: PhantomData,
                _i: PhantomData,
            })
         } else {
            let isr = self.port_driver().await.get_isr();
            Ok(Pin {
                pin_mask: self.pin_mask,
                port_driver: self.port_driver,
                irq: Some(isr),
                _m: PhantomData,
                _g: PhantomData,
                _i: PhantomData,
            })
        }
        
    }
}

impl <'a, RM, PD, IRQ, RC> Pin<'a, crate::mode::ISRInput,  PD, RM, IRQ, RC> 
where 
    PD: crate::PortDriver + crate::PortDriverISR<IRQ, RC>,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    RC: core::ops::Deref<Target = IRQ> + Clone + AsRef<IRQ> + core::borrow::Borrow<IRQ> 
{
    pub async fn enable_interrupt(&mut self, interrupt_type: crate::InterruptType) -> Result<(), PinError<PD::Error>> {
        if self.irq.is_some() { self.irq.as_ref().unwrap().register_irq(self.pin_mask, interrupt_type); }
        self.port_driver().await.configure_interrupts(self.pin_mask, 0, interrupt_type).await?;
        Ok(())
    }

    pub async fn disable_interrupt(&mut self) -> Result<(), PinError<PD::Error>> {
        if self.irq.is_some() { self.irq.as_ref().unwrap().unregister_irq(self.pin_mask); }
        self.port_driver().await.configure_interrupts(0, self.pin_mask, crate::InterruptType::Both).await?;
        Ok(())
    }

    pub async fn wait_for_interrupt(&mut self, interrupt_type: crate::InterruptType) -> Result<(), PinError<PD::Error>> {
        self.enable_interrupt(interrupt_type).await?;
        let maybe_irq = &self.irq;
        if let Some(irq) = maybe_irq {
            let maybe_irq = irq.get_irq(self.pin_mask);
            if let Some(future) = maybe_irq {
                future.await;
                Ok(())
            } else {
                todo!("No ISR available")
            }
        } else {
            todo!("No ISR available")
        }
    }
}

impl<'a, MODE, RM, PD, IRQ, RC> Pin<'a, MODE,  PD, RM, IRQ, RC>
where
    PD: crate::PortDriver + crate::PortDriverPolarity,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    RC: core::ops::Deref<Target = IRQ> + Clone + AsRef<IRQ> + core::borrow::Borrow<IRQ> 
{
    /// Turn on hardware polarity inversion for this pin.
    pub async fn into_inverted(self) -> Result<Self, PinError<PD::Error>> {
        self.port_driver().await.set_polarity(self.pin_mask, crate::Polarity::ActiveLow).await?;
        Ok(self)
    }

    /// Set hardware polarity inversion for this pin.
    pub async fn set_polarity(&mut self, polarity: crate::Polarity) -> Result<(), PinError<PD::Error>> {
        self.port_driver().await.set_polarity(self.pin_mask, polarity).await?;
        Ok(())
    }
}

impl<'a, MODE: crate::mode::HasInput, RM, PD, IRQ, RC> Pin<'a, MODE, PD, RM, IRQ, RC>
where
    PD: crate::PortDriver,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    RC: core::ops::Deref<Target = IRQ> + Clone + AsRef<IRQ> + core::borrow::Borrow<IRQ> 
{
    /// Read the pin's input state and return `true` if it is HIGH.
    pub async fn is_high(&self) -> Result<bool, PinError<PD::Error>> {
        Ok(self.port_driver().await.get(self.pin_mask, 0).await? == self.pin_mask)
    }

    /// Read the pin's input state and return `true` if it is LOW.
    pub async fn is_low(&self) -> Result<bool, PinError<PD::Error>> {
        Ok(self.port_driver().await.get(0, self.pin_mask).await? == self.pin_mask)
    }
}

impl<'a, MODE: crate::mode::HasInput, RM, PD, IRQ, RC> Pin<'a, MODE, PD, RM, IRQ, RC>
where
    PD: crate::PortDriver + crate::PortDriverPullUp,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    RC: core::ops::Deref<Target = IRQ> + Clone + AsRef<IRQ> + core::borrow::Borrow<IRQ> 
{
    /// Enable/Disable pull-up resistors for this pin.
    ///
    /// If `enable` is `true`, the pull-up resistor is enabled, otherwise the pin is configured as floating input.
    pub async fn enable_pull_up(&mut self, enable: bool) -> Result<(), PinError<PD::Error>> {
        self.port_driver().await.set_pull_up(self.pin_mask, enable).await?;
        Ok(())
    }
}

impl<'a, MODE: crate::mode::HasInput, RM, PD, IRQ, RC> Pin<'a, MODE, PD, RM, IRQ, RC>
where
    PD: crate::PortDriver + crate::PortDriverPullDown,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    RC: core::ops::Deref<Target = IRQ> + Clone + AsRef<IRQ> + core::borrow::Borrow<IRQ> 
{
    /// Enable/Disable pull-down resistors for this pin.
    ///
    /// If `enable` is `true`, the pull-down resistor is enabled, otherwise the pin is configured as floating input.
    pub async fn enable_pull_down(&mut self, enable: bool) -> Result<(), PinError<PD::Error>> {
        self.port_driver().await.set_pull_down(self.pin_mask, enable).await?;
        Ok(())
    }
}

impl<'a, MODE: crate::mode::HasInput, RM, PD, IRQ, RC> hal_digital::InputPin for Pin<'a, MODE, PD, RM, IRQ, RC>
where
    PD: crate::PortDriver + crate::PortDriverDirection,
    <PD as crate::PortDriver>::Error: core::fmt::Debug,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    RC: core::ops::Deref<Target = IRQ> + Clone + AsRef<IRQ> + core::borrow::Borrow<IRQ> 
{
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        block_on(Pin::is_high(self))
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        block_on(Pin::is_low(self))
    }
}

impl<'a, RM, PD, ISR, RC> Wait for Pin<'a, crate::mode::ISRInput, PD, RM, ISR, RC>
where
    PD: crate::PortDriver + crate::PortDriverDirection + crate::PortDriverISR<ISR, RC>,
    <PD as crate::PortDriver>::Error: core::fmt::Debug,
    RM: RawMutex,
    ISR: crate::IRQPort,
    RC: core::ops::Deref<Target = ISR> + Clone + AsRef<ISR> + core::borrow::Borrow<ISR> 
{
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.wait_for_interrupt(crate::InterruptType::High).await
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.wait_for_interrupt(crate::InterruptType::Low).await
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_interrupt(crate::InterruptType::Rising).await
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_interrupt(crate::InterruptType::Falling).await
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_interrupt(crate::InterruptType::Both).await
    }
}

impl<'a, MODE: crate::mode::HasOutput, RM, PD, IRQ, RC> Pin<'a, MODE, PD, RM, IRQ, RC>
where
    PD: crate::PortDriver,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    RC: core::ops::Deref<Target = IRQ> + Clone + AsRef<IRQ> + core::borrow::Borrow<IRQ> 
{
    /// Set the pin's output state to HIGH.
    ///
    /// Note that this can have different electrical meanings depending on the port-expander chip.
    pub async fn set_high(&mut self) -> Result<(), PinError<PD::Error>> {
        self.port_driver().await.set(self.pin_mask, 0).await?;
        Ok(())
    }

    /// Set the pin's output state to LOW.
    ///
    /// Note that this can have different electrical meanings depending on the port-expander chip.
    pub async fn set_low(&mut self) -> Result<(), PinError<PD::Error>> {
        self.port_driver().await.set(0, self.pin_mask).await?;
        Ok(())
    }

    /// Return `true` if the pin's output state is HIGH.
    ///
    /// This method does **not** read the pin's electrical state.
    pub fn is_set_high(&self) -> Result<bool, PinError<PD::Error>> {
        Ok(self.port_driver.try_lock()
            .map(|mut pd| pd.is_set(self.pin_mask, 0)).unwrap()? == self.pin_mask)
    }

    /// Return `true` if the pin's output state is LOW.
    ///
    /// This method does **not** read the pin's electrical state.
    pub fn is_set_low(&self) -> Result<bool, PinError<PD::Error>> {
        Ok(self.port_driver.try_lock()
            .map(|mut pd| pd.is_set(0, self.pin_mask)).unwrap()? == self.pin_mask)
    }

    /// Toggle the pin's output state.
    pub async fn toggle(&mut self) -> Result<(), PinError<PD::Error>> {
        self.port_driver().await.toggle(self.pin_mask).await?;
        Ok(())
    }
}

impl<'a, MODE: crate::mode::HasOutput, RM, PD, IRQ, RC> hal_digital::OutputPin for Pin<'a, MODE, PD, RM, IRQ, RC>
where
    PD: crate::PortDriver + crate::PortDriverDirection,
    <PD as crate::PortDriver>::Error: core::fmt::Debug,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    RC: core::ops::Deref<Target = IRQ> + Clone + AsRef<IRQ> + core::borrow::Borrow<IRQ> 
{
    fn set_low(&mut self) -> Result<(), Self::Error> {
        block_on(Pin::set_low(self))
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        block_on(Pin::set_high(self))
    }
}

impl<'a, MODE: crate::mode::HasOutput, RM, PD, IRQ, RC> hal_digital::StatefulOutputPin
    for Pin<'a, MODE, PD, RM, IRQ, RC>
where
    PD: crate::PortDriver + crate::PortDriverDirection,
    <PD as crate::PortDriver>::Error: core::fmt::Debug,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    RC: core::ops::Deref<Target = IRQ> + Clone + AsRef<IRQ> + core::borrow::Borrow<IRQ> 
{
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Pin::is_set_high(self)
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Pin::is_set_low(self)
    }

    fn toggle(&mut self) -> Result<(), Self::Error> {
        block_on(Pin::toggle(self))
    }
}