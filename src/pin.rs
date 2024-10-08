
use core::marker::PhantomData;
use embassy_futures::block_on;
use embassy_sync::{blocking_mutex::raw::RawMutex, mutex::{Mutex, MutexGuard}};
use embedded_hal::digital::{self as hal_digital, ErrorType};
use embedded_hal_async::digital::Wait;





/// Representation of a port-expander pin.
///
/// `Pin` is not constructed directly, this type is created by instanciating a port-expander and
/// then getting access to all its pins using the `.split()` method.
pub struct Pin<'a, MODE, PD, RM: RawMutex, IRQRC> {
    pin_mask: u32,
    port_driver: &'a Mutex<RM, PD>,
    irq: Option<IRQRC>,
    _m: PhantomData<MODE>,
}

impl<'a, MODE, RM, PD, IRQ, IRQRC> Pin<'a, MODE, PD, RM, IRQRC>
where
    PD: crate::PortDriver,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    IRQRC: core::ops::Deref<Target = IRQ>
{
    pub(crate) fn new(pin_number: u8, port_driver: &'a Mutex<RM, PD>) -> Self {
        assert!(pin_number < 32);
        Self {
            pin_mask: 1 << pin_number,
            port_driver,
            irq: None,
            _m: PhantomData,
        }
    }

    /// Read the pin mask for this pin.
    pub fn pin_mask(&self) -> u32 {
        self.pin_mask
    }
    /// Access the port driver in an async context.
    pub async fn port_driver(&self) -> MutexGuard<RM, PD> {
        self.port_driver.lock().await
    }

    /// Access the port driver directly, if needed, to run a function using it.
    /// Due to the lack of async closures, this has limitations.
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
#[derive(Debug, defmt::Format)]
pub enum PinError<PDE> {
    DriverError(PDE),
    NoISR,
}

impl<PDE> PinError<PDE> {
    /// The upstream port driver error that occurred
    pub fn driver_error(&self) -> &PDE {
        match self {
            PinError::DriverError(e) => e,
            PinError::NoISR => panic!("No ISR available"),
        }
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
        Self::DriverError(value)
    }
}

impl<'a, MODE, RM, PD, ISR> ErrorType for Pin<'a, MODE, PD, RM, ISR>
where
    PD: crate::PortDriver + crate::PortDriverDirection,
    RM: RawMutex,
    PD::Error: core::fmt::Debug,
    ISR: crate::IRQPort {
    type Error = PinError<PD::Error>;
}

impl<'a, MODE, RM, PD, IRQ, IRQRC> Pin<'a, MODE, PD, RM, IRQRC>
where
    PD: crate::PortDriver + crate::PortDriverDirection,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    IRQRC: core::ops::Deref<Target = IRQ> + Clone
{

    /// Configure this pin into an input.
    ///
    /// The exact electrical details depend on the port-expander device which is used.
    pub async fn into_input(&self) -> Result<Pin<'a, crate::mode::Input, PD, RM, IRQRC>, PinError<PD::Error>> {
        self.port_driver().await.set_direction(self.pin_mask, crate::Direction::Input, false).await?;
        Ok(Pin {
            pin_mask: self.pin_mask,
            port_driver: self.port_driver,
            irq: self.irq.clone(),
            _m: PhantomData,
        })
    }

    /// Configure this pin into an output with an initial LOW state.
    ///
    /// The LOW state is, as long as the port-expander chip allows this, entered without any
    /// electrical glitch.
    pub async fn into_output(&self) -> Result<Pin<'a, crate::mode::Output, PD, RM, IRQRC>, PinError<PD::Error>> {
        self.port_driver().await.set_direction(self.pin_mask, crate::Direction::Output, false).await?;
        Ok(Pin {
            pin_mask: self.pin_mask,
            port_driver: self.port_driver,
            irq: self.irq.clone(),
            _m: PhantomData,
        })
    }

    /// Configure this pin into an output with an initial HIGH state.
    ///
    /// The HIGH state is, as long as he port-expander chip allows this, entered without any
    /// electrical glitch.
    pub async fn into_output_high(
        &self,
    ) -> Result<Pin<'a, crate::mode::Output, PD, RM, IRQRC>, PinError<PD::Error>> {
        self.port_driver().await.set_direction(self.pin_mask, crate::Direction::Output, true).await?;
        Ok(Pin {
            pin_mask: self.pin_mask,
            port_driver: self.port_driver,
            irq: self.irq.clone(),
            _m: PhantomData,
        })
    }
}

impl <'a, MODE: crate::mode::HasInterrupt, RM, PD, IRQ, IRQRC> Pin<'a, MODE,  PD, RM, IRQRC> 
where 
    PD: crate::PortDriver + crate::PortDriverISR<IRQ, IRQRC>,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    IRQRC: core::ops::Deref<Target = IRQ> + Clone
{
    /// Configure this pin into an input with interrupt support.
    pub async fn into_isr_pin(&self) -> Result<Pin<'a, crate::mode::ISRInput, PD, RM, IRQRC>, PinError<PD::Error>> {
        if self.irq.is_some() { 
            Ok(Pin {
                pin_mask: self.pin_mask,
                port_driver: self.port_driver,
                irq: self.irq.clone(),
                _m: PhantomData,
            })
         } else {
            let isr = self.port_driver().await.get_isr();
            Ok(Pin {
                pin_mask: self.pin_mask,
                port_driver: self.port_driver,
                irq: Some(isr),
                _m: PhantomData,
            })
        }
        
    }
}

impl <'a, RM, PD, IRQ, IRQRC> Pin<'a, crate::mode::ISRInput, PD, RM, IRQRC> 
where 
    PD: crate::PortDriver + crate::PortDriverISR<IRQ, IRQRC>,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    IRQRC: core::ops::Deref<Target = IRQ>   
{
    /// Listen for the specified interrupt type on this pin.
    pub async fn enable_interrupt(&mut self, interrupt_type: crate::InterruptType) -> Result<(), PinError<PD::Error>> {
        if self.irq.is_some() { self.irq.as_ref().unwrap().register_irq(self.pin_mask, interrupt_type); }
        self.port_driver().await.configure_interrupts(self.pin_mask, 0, interrupt_type).await?;
        Ok(())
    }

    /// Stop listening for interrupts on this pin.
    pub async fn disable_interrupt(&mut self) -> Result<(), PinError<PD::Error>> {
        if self.irq.is_some() { self.irq.as_ref().unwrap().unregister_irq(self.pin_mask); }
        self.port_driver().await.configure_interrupts(0, self.pin_mask, crate::InterruptType::Both).await?;
        Ok(())
    }

    /// Wait for an interrupt on this pin. Underlies the `embedded_hal_async::digital::Wait` trait implementation.
    pub async fn wait_for_interrupt(&mut self, interrupt_type: crate::InterruptType) -> Result<(), PinError<PD::Error>> {
        self.enable_interrupt(interrupt_type).await?;
        let maybe_irq = &self.irq;
        if let Some(irq) = maybe_irq {
            let maybe_irq = irq.get_irq(self.pin_mask);
            if let Some(future) = maybe_irq {
                future.await;
                Ok(())
            } else {
                Err(PinError::NoISR)
            }
        } else {
            Err(PinError::NoISR)
        }
    }

    /// gets the next interrupt, regardless of the interrupt type
    pub async fn get_next_interrupt(&mut self) -> Result<crate::InterruptType, PinError<PD::Error>> {
        let maybe_irq = &self.irq;
        if let Some(irq) = maybe_irq {
            let maybe_irq = irq.get_irq(self.pin_mask);
            if let Some(future) = maybe_irq {
                let interrupt = future.await;
                
                Ok(interrupt)
            } else {
                Err(PinError::NoISR)
            }
        } else {
            Err(PinError::NoISR)
        }
    }
}

impl<'a, MODE, RM, PD, IRQ, IRQRC> Pin<'a, MODE,  PD, RM, IRQRC>
where
    PD: crate::PortDriver + crate::PortDriverPolarity,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    IRQRC: core::ops::Deref<Target = IRQ>
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

impl<'a, MODE: crate::mode::HasInput, RM, PD, IRQ, IRQRC> Pin<'a, MODE, PD, RM, IRQRC>
where
    PD: crate::PortDriver,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    IRQRC: core::ops::Deref<Target = IRQ>
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

impl<'a, MODE: crate::mode::HasInput, RM, PD, IRQ, IRQRC> Pin<'a, MODE, PD, RM, IRQRC>
where
    PD: crate::PortDriver + crate::PortDriverPullUp,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    IRQRC: core::ops::Deref<Target = IRQ>
{
    /// Enable/Disable pull-up resistors for this pin.
    ///
    /// If `enable` is `true`, the pull-up resistor is enabled, otherwise the pin is configured as floating input.
    pub async fn enable_pull_up(&mut self, enable: bool) -> Result<(), PinError<PD::Error>> {
        self.port_driver().await.set_pull_up(self.pin_mask, enable).await?;
        Ok(())
    }
}

impl<'a, MODE: crate::mode::HasInput, RM, PD, IRQ, IRQRC> Pin<'a, MODE, PD, RM, IRQRC>
where
    PD: crate::PortDriver + crate::PortDriverPullDown,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    IRQRC: core::ops::Deref<Target = IRQ>
{
    /// Enable/Disable pull-down resistors for this pin.
    ///
    /// If `enable` is `true`, the pull-down resistor is enabled, otherwise the pin is configured as floating input.
    pub async fn enable_pull_down(&mut self, enable: bool) -> Result<(), PinError<PD::Error>> {
        self.port_driver().await.set_pull_down(self.pin_mask, enable).await?;
        Ok(())
    }
}

impl<'a, MODE: crate::mode::HasInput, RM, PD, IRQ, IRQRC> hal_digital::InputPin for Pin<'a, MODE, PD, RM, IRQRC>
where
    PD: crate::PortDriver + crate::PortDriverDirection,
    <PD as crate::PortDriver>::Error: core::fmt::Debug,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    IRQRC: core::ops::Deref<Target = IRQ> + Clone
{
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        block_on(Pin::is_high(self))
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        block_on(Pin::is_low(self))
    }
}

impl<'a, RM, PD, ISR, ISRRC> Wait for Pin<'a, crate::mode::ISRInput, PD, RM, ISRRC>
where
    PD: crate::PortDriver + crate::PortDriverDirection + crate::PortDriverISR<ISR, ISRRC>,
    <PD as crate::PortDriver>::Error: core::fmt::Debug,
    RM: RawMutex,
    ISR: crate::IRQPort,
    ISRRC: core::ops::Deref<Target = ISR> + Clone
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

impl<'a, MODE: crate::mode::HasOutput, RM, PD, IRQ, IRQRC> Pin<'a, MODE, PD, RM, IRQRC>
where
    PD: crate::PortDriver,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    IRQRC: core::ops::Deref<Target = IRQ>
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

impl<'a, MODE: crate::mode::HasOutput, RM, PD, IRQ, IRQRC> hal_digital::OutputPin for Pin<'a, MODE, PD, RM, IRQRC>
where
    PD: crate::PortDriver + crate::PortDriverDirection,
    <PD as crate::PortDriver>::Error: core::fmt::Debug,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    IRQRC: core::ops::Deref<Target = IRQ> + Clone
{
    fn set_low(&mut self) -> Result<(), Self::Error> {
        block_on(Pin::set_low(self))
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        block_on(Pin::set_high(self))
    }
}

impl<'a, MODE: crate::mode::HasOutput, RM, PD, IRQ, IRQRC> hal_digital::StatefulOutputPin
    for Pin<'a, MODE, PD, RM, IRQRC>
where
    PD: crate::PortDriver + crate::PortDriverDirection,
    <PD as crate::PortDriver>::Error: core::fmt::Debug,
    RM: RawMutex,
    IRQ: crate::IRQPort,
    IRQRC: core::ops::Deref<Target = IRQ> + Clone
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