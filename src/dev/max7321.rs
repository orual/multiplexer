//! Support for the Maxim 7321 I2C 8-Port Open Drain port expander
use core::{borrow::Borrow, ops::Deref};
use embassy_sync::{blocking_mutex::raw::RawMutex, mutex::Mutex};

use crate::IRQPort;

/// MAX7321 "I2C Port Expander with 8 Open-Drain I/Os"
pub struct Max7321<M>(M);

/// MAX7321 "I2C Port Expander with 8 Open-Drain I/Os"
impl<I2C, RM, IRQ, IRQRC> Max7321<Mutex<RM, Driver<I2C, IRQ, IRQRC>>>
where
    I2C: crate::I2cBus,
    RM: RawMutex,
    IRQ: IRQPort,
    IRQRC: Deref<Target = IRQ> + Clone + AsRef<IRQ> + Borrow<IRQ> + ?Sized
{
    /// Create a new MAX7321 driver instance
    pub fn new(i2c: I2C, a3: bool, a2: bool, a1: bool, a0: bool) -> Self {
        Self::with_mutex(i2c, a3, a2, a1, a0)
    }
}

impl<I2C, RM, IRQ, IRQRC> Max7321<Mutex<RM, Driver<I2C, IRQ, IRQRC>>>
where
    I2C: crate::I2cBus,
    RM: RawMutex,
    IRQ: IRQPort,
    IRQRC: Deref<Target = IRQ> + Clone + AsRef<IRQ> + Borrow<IRQ> + ?Sized
{
    /// Create a new MAX7321 driver instance with a mutex
    pub fn with_mutex(i2c: I2C, a3: bool, a2: bool, a1: bool, a0: bool) -> Self {
        Self(Mutex::new(Driver::new(i2c, a3, a2, a1, a0)))
    }

    /// Split the MAX7321 driver instance into its individual pins
    pub fn split(&mut self) -> Parts<'_, I2C, RM, IRQ, IRQRC> {
        Parts {
            p0: crate::Pin::new(0, &self.0),
            p1: crate::Pin::new(1, &self.0),
            p2: crate::Pin::new(2, &self.0),
            p3: crate::Pin::new(3, &self.0),
            p4: crate::Pin::new(4, &self.0),
            p5: crate::Pin::new(5, &self.0),
            p6: crate::Pin::new(6, &self.0),
            p7: crate::Pin::new(7, &self.0),
        }
    }
}

/// Pins of the MAX7321
#[allow(missing_docs)]
pub struct Parts<'a, I2C, RM, IRQ, IRQRC>
where
    I2C: crate::I2cBus,
    RM: RawMutex,
    IRQ: IRQPort
{
    pub p0: crate::Pin<'a, crate::mode::QuasiBidirectional, Driver<I2C, IRQ, IRQRC>, RM, IRQRC>,
    pub p1: crate::Pin<'a, crate::mode::QuasiBidirectional, Driver<I2C, IRQ, IRQRC>, RM, IRQRC>,
    pub p2: crate::Pin<'a, crate::mode::QuasiBidirectional, Driver<I2C, IRQ, IRQRC>, RM, IRQRC>,
    pub p3: crate::Pin<'a, crate::mode::QuasiBidirectional, Driver<I2C, IRQ, IRQRC>, RM, IRQRC>,
    pub p4: crate::Pin<'a, crate::mode::QuasiBidirectional, Driver<I2C, IRQ, IRQRC>, RM, IRQRC>,
    pub p5: crate::Pin<'a, crate::mode::QuasiBidirectional, Driver<I2C, IRQ, IRQRC>, RM, IRQRC>,
    pub p6: crate::Pin<'a, crate::mode::QuasiBidirectional, Driver<I2C, IRQ, IRQRC>, RM, IRQRC>,
    pub p7: crate::Pin<'a, crate::mode::QuasiBidirectional, Driver<I2C, IRQ, IRQRC>, RM, IRQRC>,
}

/// MAX7321 generic driver
pub struct Driver<I2C, IRQ, IRQRC> {
    i2c: I2C,
    out: u8,
    addr: u8,
    _irq_port: Option<IRQRC>,
    _iq: core::marker::PhantomData<IRQ>,
}

#[allow(missing_docs)]
impl<I2C, IRQ, IRQRC> Driver<I2C, IRQ, IRQRC> {
    pub fn new(i2c: I2C, a3: bool, a2: bool, a1: bool, a0: bool) -> Self {
        let addr = 0x60 | ((a3 as u8) << 3) | ((a2 as u8) << 2) | ((a1 as u8) << 1) | (a0 as u8);
        Self {
            i2c,
            out: 0xff,
            addr,
            _irq_port: None,
            _iq: core::marker::PhantomData,
        }
    }
}

impl<I2C: crate::I2cBus, IRQ, IRQRC> crate::PortDriver for Driver<I2C, IRQ, IRQRC> {
    type Error = I2C::BusError;

    async fn set(&mut self, mask_high: u32, mask_low: u32) -> Result<(), Self::Error> {
        self.out |= mask_high as u8;
        self.out &= !mask_low as u8;
        self.i2c.write(self.addr, &[self.out]).await?;
        Ok(())
    }

    fn is_set(&mut self, mask_high: u32, mask_low: u32) -> Result<u32, Self::Error> {
        Ok(((self.out as u32) & mask_high) | (!(self.out as u32) & mask_low))
    }

    async fn get(&mut self, mask_high: u32, mask_low: u32) -> Result<u32, Self::Error> {
        let mut buf = [0x00];
        self.i2c.read(self.addr, &mut buf).await?;
        let in_ = buf[0] as u32;
        Ok((in_ & mask_high) | (!in_ & mask_low))
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal_mock::eh1::i2c as mock_i2c;

    #[test]
    fn max7321() {
        let expectations = [
            mock_i2c::Transaction::write(0b01101101, vec![0b11111111]),
            mock_i2c::Transaction::write(0b01101101, vec![0b11111011]),
            mock_i2c::Transaction::read(0b01101101, vec![0b01000000]),
            mock_i2c::Transaction::read(0b01101101, vec![0b10111111]),
        ];
        let mut bus = mock_i2c::Mock::new(&expectations);

        let mut max = super::Max7321::new(bus.clone(), true, true, false, true);
        let mut max_pins = max.split();

        max_pins.p2.set_high().unwrap();
        max_pins.p2.set_low().unwrap();

        assert!(max_pins.p6.is_high().unwrap());
        assert!(max_pins.p6.is_low().unwrap());

        bus.done();
    }
}
