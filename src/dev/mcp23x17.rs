//! Support for the `MCP23017` and `MCP23S17` "16-Bit I/O Expander with Serial Interface"
//!
//! Datasheet: <https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf>
//!
//! The MCP23x17 offers two eight-bit GPIO ports.  It has three
//! address pins, so eight devices can coexist on an I2C bus.
//!
//! Each port has an interrupt, which can be configured to work
//! together or independently.
//!
//! When passing 16-bit values to this driver, the upper byte corresponds to port
//! B (pins 7..0) and the lower byte corresponds to port A (pins 7..0).
use core::{borrow::Borrow, ops::Deref};

use embassy_futures::select::select;
use embassy_sync::{blocking_mutex::raw::RawMutex, mutex::Mutex};

use crate::{isr::{ExtIPin, ISRPort, IrqPort}, I2cExt, IRQPort};

/// `MCP23x17` "16-Bit I/O Expander with Serial Interface" with I2C or SPI interface
pub struct Mcp23x17<M>(M);

impl<I2C, RM, ISRRC, ISR, IRQRC, IRQ> Mcp23x17<Mutex<RM, Driver<Mcp23017Bus<I2C>, ISRRC, RM, ISR, IRQRC, IRQ>>>
where
    I2C: crate::I2cBus,
    RM: RawMutex,
    ISR: ISRPort,
    ISRRC: Deref<Target = ISR> + Clone + AsRef<ISR> + Borrow<ISR> + ?Sized,
    IRQ: IRQPort,
    IRQRC: Deref<Target = IRQ> + Clone + AsRef<IRQ> + Borrow<IRQ> + ?Sized
{
    /// Create a new instance of the MCP23017 with I2C interface
    pub fn new_mcp23017(bus: I2C, isr: ISRRC, irq: IRQRC, a0: bool, a1: bool, a2: bool) -> Self {
        Self::with_mutex(Mcp23017Bus(bus), isr, irq, a0, a1, a2)
    }
}

impl<SPI, RM, ISRRC, ISR, IRQRC, IRQ> Mcp23x17<Mutex<RM, Driver<Mcp23S17Bus<SPI>, ISRRC, RM, ISR, IRQRC, IRQ>>>
where
    SPI: crate::SpiBus,
    RM: RawMutex,
    ISR: ISRPort,
    ISRRC: Deref<Target = ISR> + Clone + AsRef<ISR> + Borrow<ISR> + ?Sized,
    IRQ: IRQPort,
    IRQRC: Deref<Target = IRQ> + Clone + AsRef<IRQ> + Borrow<IRQ> + ?Sized
{
    /// Create a new instance of the MCP23S17 with SPI interface
    pub fn new_mcp23s17(bus: SPI, isr: ISRRC, irq: IRQRC) -> Self {
        Self::with_mutex(Mcp23S17Bus(bus),  isr, irq, false, false, false)
    }
}

/// Interrupt output pins for the MCP23x17 chips (INTA and INTB)
/// Pass in the `ExtIPin` instances for the interrupt pins.
/// Each should contain a host GPIO pin which implements `embedded_hal_async::digital::Wait` and is connected to the equivalent pin on the MCP..
pub struct McpExtIPins<IA, IB>(IA, IB);

impl<RM, W, E> ISRPort for McpExtIPins<ExtIPin<Mutex<RM, W>>, ExtIPin<Mutex<RM, W>>> 
where 
RM: RawMutex,
E: core::fmt::Debug,
W: embedded_hal_async::digital::Wait<Error = E> 
{
    async fn wait_for_interrupt(&self) {
        let (mut a, mut b) = (self.0.0.lock().await, self.1.0.lock().await);
        let (a, b) = (a.wait_for_any_edge(), b.wait_for_any_edge());
        select(a, b).await;
    }
}

impl<'a, B, RM, ISR, ISRRC, IRQRC, IRQ> Mcp23x17<Mutex<RM, Driver<B, ISRRC, RM, ISR, IRQRC, IRQ>>>
where
    B: Mcp23x17Bus,
    RM: RawMutex,
    ISR: ISRPort,
    ISRRC: Deref<Target = ISR> + Clone + AsRef<ISR> + Borrow<ISR> + ?Sized,
    IRQ: IRQPort,
    IRQRC: Deref<Target = IRQ> + Clone + AsRef<IRQ> + Borrow<IRQ> + ?Sized
{
    /// Create a new MCP23x17 driver instance with a mutex
    pub fn with_mutex(bus: B, isr: ISRRC, irq: IRQRC, a0: bool, a1: bool, a2: bool) -> Self {
        Self(Mutex::new(Driver::new(bus, isr, irq, a0, a1, a2)))
    }
    
    /// Split the MCP23x17 driver instance into its individual pins
    pub fn split(&'a mut self) -> Parts<'a, B, ISR, IRQ, RM, ISRRC, IRQRC, Driver<B, ISRRC, RM, ISR, IRQRC, IRQ>> 
    where 
        
    {
        Parts {
            gpa0: crate::Pin::new(0, &self.0),
            gpa1: crate::Pin::new(1, &self.0),
            gpa2: crate::Pin::new(2, &self.0),
            gpa3: crate::Pin::new(3, &self.0),
            gpa4: crate::Pin::new(4, &self.0),
            gpa5: crate::Pin::new(5, &self.0),
            gpa6: crate::Pin::new(6, &self.0),
            gpa7: crate::Pin::new(7, &self.0),
            gpb0: crate::Pin::new(8, &self.0),
            gpb1: crate::Pin::new(9, &self.0),
            gpb2: crate::Pin::new(10, &self.0),
            gpb3: crate::Pin::new(11, &self.0),
            gpb4: crate::Pin::new(12, &self.0),
            gpb5: crate::Pin::new(13, &self.0),
            gpb6: crate::Pin::new(14, &self.0),
            gpb7: crate::Pin::new(15, &self.0),
            _b: core::marker::PhantomData,
            _i: core::marker::PhantomData,
            _r: core::marker::PhantomData,
        }
    }
}

/// Pins of the MCP23x17
#[allow(missing_docs)]
pub struct Parts<'a, B, ISR, IRQ, RM, ISRRC, IRQRC, PD = Driver<B, ISRRC, RM, ISR, IRQRC, IRQ>>
where
    B: Mcp23x17Bus,
    RM: RawMutex,
    ISR: ISRPort,
    IRQ: IRQPort,
    ISRRC: Deref<Target = ISR> + Clone + AsRef<ISR> + Borrow<ISR> + ?Sized,
    IRQRC: Deref<Target = IRQ> + Clone + AsRef<IRQ> + Borrow<IRQ> + ?Sized
{
    pub gpa0: crate::Pin<'a, crate::mode::ISRInput, PD, RM, IRQ, IRQRC>,
    pub gpa1: crate::Pin<'a, crate::mode::ISRInput, PD, RM, IRQ, IRQRC>,
    pub gpa2: crate::Pin<'a, crate::mode::ISRInput, PD, RM, IRQ, IRQRC>,
    pub gpa3: crate::Pin<'a, crate::mode::ISRInput, PD, RM, IRQ, IRQRC>,
    pub gpa4: crate::Pin<'a, crate::mode::ISRInput, PD, RM, IRQ, IRQRC>,
    pub gpa5: crate::Pin<'a, crate::mode::ISRInput, PD, RM, IRQ, IRQRC>,
    pub gpa6: crate::Pin<'a, crate::mode::ISRInput, PD, RM, IRQ, IRQRC>,
    pub gpa7: crate::Pin<'a, crate::mode::ISRInput, PD, RM, IRQ, IRQRC>,
    pub gpb0: crate::Pin<'a, crate::mode::ISRInput, PD, RM, IRQ, IRQRC>,
    pub gpb1: crate::Pin<'a, crate::mode::ISRInput, PD, RM, IRQ, IRQRC>,
    pub gpb2: crate::Pin<'a, crate::mode::ISRInput, PD, RM, IRQ, IRQRC>,
    pub gpb3: crate::Pin<'a, crate::mode::ISRInput, PD, RM, IRQ, IRQRC>,
    pub gpb4: crate::Pin<'a, crate::mode::ISRInput, PD, RM, IRQ, IRQRC>,
    pub gpb5: crate::Pin<'a, crate::mode::ISRInput, PD, RM, IRQ, IRQRC>,
    pub gpb6: crate::Pin<'a, crate::mode::ISRInput, PD, RM, IRQ, IRQRC>,
    pub gpb7: crate::Pin<'a, crate::mode::ISRInput, PD, RM, IRQ, IRQRC>,
    _b: core::marker::PhantomData<B>,
    _i: core::marker::PhantomData<ISR>,
    _r: core::marker::PhantomData<ISRRC>,
}


#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// N.B.: These values are for BANK=0, which is the reset state of
/// the chip (and this driver does not change).
///
/// For all registers, the reset value is 0x00, except for
/// IODIR{A,B} which are 0xFF (making all pins inputs) at reset.
enum Regs {
    /// IODIR: input/output direction: 0=output; 1=input
    IODIRA = 0x00,
    /// IPOL: input polarity: 0=register values match input pins; 1=opposite
    IPOLA = 0x02,
    /// GPINTEN: interrupt-on-change: 0=disable; 1=enable
    GPINTENA = 0x04,
    /// DEFVAL: default values for interrupt-on-change
    DEFVALA = 0x06,
    /// INTCON: interrupt-on-change config: 0=compare to previous pin value;
    ///   1=compare to corresponding bit in DEFVAL
    INTCONA = 0x08,
    /// IOCON: configuration register
    /// - Pin 7: BANK (which driver assumes stays 0)
    /// - Pin 6: MIRROR: if enabled, INT{A,B} are logically ORed; an interrupt on either
    ///          port will cause both pins to activate
    /// - Pin 5: SEQOP: controls the incrementing function of the address pointer
    /// - Pin 4: DISSLW: disables slew rate control on SDA
    /// - Pin 3: HAEN: no effect on MCP23017, enables address pins on MCP23S17
    /// - Pin 2: ODR: interrupt pins are 0=active-driver outputs (INTPOL sets polarity)
    ///          or 1=open-drain outputs (overrides INTPOL)
    /// - Pin 1: INTPOL: interrupt pin is 0=active-low or 1=active-high
    /// - Pin 0: unused
    IOCONA = 0x0a,
    /// GPPU: GPIO pull-ups: enables weak internal pull-ups on each pin (when configured
    ///   as an input)
    GPPUA = 0x0c,
    /// INTF: interrupt flags: 0=no interrupt pending; 1=corresponding pin caused interrupt
    INTFA = 0x0e,
    /// INTCAP: interrupt captured value: reflects value of each pin at the time that they
    ///   caused an interrupt
    INTCAPA = 0x10,
    /// GPIO: reflects logic level on pins
    GPIOA = 0x12,
    /// OLAT: output latches: sets state for pins configured as outputs
    OLATA = 0x14,
    /// IODIR: input/output direction: 0=output; 1=input
    IODIRB = 0x01,
    /// IPOL: input polarity: 0=register values match input pins; 1=opposite
    IPOLB = 0x03,
    /// GPINTEN: interrupt-on-change: 0=disable; 1=enable
    GPINTENB = 0x05,
    /// DEFVAL: default values for interrupt-on-change
    DEFVALB = 0x07,
    /// INTCON: interrupt-on-change config: 0=compare to previous pin value;
    ///   1=compare to corresponding bit in DEFVAL
    INTCONB = 0x09,
    /// IOCON: configuration register
    /// - Pin 7: BANK (which driver assumes stays 0)
    /// - Pin 6: MIRROR: if enabled, INT{A,B} are logically ORed; an interrupt on either
    ///          port will cause both pins to activate
    /// - Pin 5: SEQOP: controls the incrementing function of the address pointer
    /// - Pin 4: DISSLW: disables slew rate control on SDA
    /// - Pin 3: HAEN: no effect on MCP23017, enables address pins on MCP23S17
    /// - Pin 2: ODR: interrupt pins are 0=active-driver outputs (INTPOL sets polarity)
    ///          or 1=open-drain outputs (overrides INTPOL)
    /// - Pin 1: INTPOL: interrupt pin is 0=active-low or 1=active-high
    /// - Pin 0: unused
    IOCONB = 0x0b,
    /// GPPU: GPIO pull-ups: enables weak internal pull-ups on each pin (when configured
    ///   as an input)
    GPPUB = 0x0d,
    /// INTF: interrupt flags: 0=no interrupt pending; 1=corresponding pin caused interrupt
    INTFB = 0x0f,
    /// INTCAP: interrupt captured value: reflects value of each pin at the time that they
    ///   caused an interrupt
    INTCAPB = 0x11,
    /// GPIO: reflects logic level on pins
    GPIOB = 0x13,
    /// OLAT: output latches: sets state for pins configured as outputs
    OLATB = 0x15,
}

impl From<Regs> for u8 {
    fn from(r: Regs) -> u8 {
        r as u8
    }
}

/// Driver for the MCP23017 and MCP23S17
pub struct Driver<B, ISRRC, RM: RawMutex, ISR, IRQRC, IRQ = IrqPort<RM, 16>> {
    bus: B,
    out: u16,
    addr: u8,
    isr: ISRRC,
    irq_port: IRQRC,
    _i: core::marker::PhantomData<ISR>,
    _iq: core::marker::PhantomData<IRQ>,
    _r: core::marker::PhantomData<RM>,
}

#[allow(missing_docs)]
impl<'a, B, ISRRC, RM, ISR, IRQRC, IRQ> Driver<B, ISRRC, RM, ISR, IRQRC, IRQ> 
where 
    RM: RawMutex,
    ISR: ISRPort,
    IRQ: IRQPort,
    ISRRC: Deref<Target = ISR> + Clone + AsRef<ISR> + Borrow<ISR>,
    IRQRC: Deref<Target = IRQ> + Clone + AsRef<IRQ> + Borrow<IRQ>  
{
    pub fn new(bus: B, isr: ISRRC, irq: IRQRC, a0: bool, a1: bool, a2: bool) -> Self {
        let addr = 0x20 | ((a2 as u8) << 2) | ((a1 as u8) << 1) | (a0 as u8);
        Self {
            bus,
            out: 0x0000,
            addr,
            isr,
            irq_port: irq,
            _i: core::marker::PhantomData,
            _iq: core::marker::PhantomData,
            _r: core::marker::PhantomData,
        }
    }
}

impl<B: Mcp23x17Bus, ISRRC, RM, ISR, IRQRC, IRQ> crate::PortDriver for Driver<B, ISRRC, RM, ISR, IRQRC, IRQ> 
where 
    RM: RawMutex,
    ISR: crate::isr::ISRPort,
    ISRRC: Deref<Target = ISR> + Clone + AsRef<ISR> + Borrow<ISR> 
{
    type Error = B::BusError;

    async fn set(&mut self, mask_high: u32, mask_low: u32) -> Result<(), Self::Error> {
        self.out |= mask_high as u16;
        self.out &= !mask_low as u16;
        if (mask_high | mask_low) & 0x00FF != 0 {
            self.bus
                .write_reg(self.addr, Regs::GPIOA, (self.out & 0xFF) as u8).await?;
        }
        if (mask_high | mask_low) & 0xFF00 != 0 {
            self.bus
                .write_reg(self.addr, Regs::GPIOB, (self.out >> 8) as u8).await?;
        }
        Ok(())
    }

    fn is_set(&mut self, mask_high: u32, mask_low: u32) -> Result<u32, Self::Error> {
        Ok(((self.out as u32) & mask_high) | (!(self.out as u32) & mask_low))
    }

    async fn get(&mut self, mask_high: u32, mask_low: u32) -> Result<u32, Self::Error> {
        let io0 = if (mask_high | mask_low) & 0x00FF != 0 {
            self.bus.read_reg(self.addr, Regs::GPIOA).await?
        } else {
            0
        };
        let io1 = if (mask_high | mask_low) & 0xFF00 != 0 {
            self.bus.read_reg(self.addr, Regs::GPIOB).await?
        } else {
            0
        };
        let in_ = ((io1 as u32) << 8) | io0 as u32;
        Ok((in_ & mask_high) | (!in_ & mask_low))
    }
}

impl<B: Mcp23x17Bus, ISRRC, RM, ISR, IRQRC, IRQ> crate::PortDriverDirection for Driver<B, ISRRC, RM, ISR, IRQRC, IRQ>
where 
    RM: RawMutex,
    ISR: ISRPort,
    ISRRC: Deref<Target = ISR> + Clone + AsRef<ISR> + Borrow<ISR> 
{
    async fn set_direction(
        &mut self,
        mask: u32,
        dir: crate::Direction,
        _state: bool,
    ) -> Result<(), Self::Error> {
        let (mask_set, mask_clear) = match dir {
            crate::Direction::Input => (mask as u16, 0),
            crate::Direction::Output => (0, mask as u16),
        };
        if mask & 0x00FF != 0 {
            self.bus.update_reg(
                self.addr,
                Regs::IODIRA,
                (mask_set & 0xFF) as u8,
                (mask_clear & 0xFF) as u8,
            ).await?;
        }
        if mask & 0xFF00 != 0 {
            self.bus.update_reg(
                self.addr,
                Regs::IODIRB,
                (mask_set >> 8) as u8,
                (mask_clear >> 8) as u8,
            ).await?;
        }
        Ok(())
    }
}

impl<B: Mcp23x17Bus, RC, RM, ISR, IRQRC, IRQ> crate::PortDriverPullUp for Driver<B, RC, RM, ISR, IRQRC, IRQ>
where 
    RM: RawMutex,
    ISR: ISRPort,
    RC: core::ops::Deref<Target = ISR> + Clone + AsRef<ISR> + core::borrow::Borrow<ISR>  
{
    async fn set_pull_up(&mut self, mask: u32, enable: bool) -> Result<(), Self::Error> {
        let (mask_set, mask_clear) = match enable {
            true => (mask as u16, 0),
            false => (0, mask as u16),
        };
        if mask & 0x00FF != 0 {
            self.bus.update_reg(
                self.addr,
                Regs::GPPUA,
                (mask_set & 0xFF) as u8,
                (mask_clear & 0xFF) as u8,
            ).await?;
        }
        if mask & 0xFF00 != 0 {
            self.bus.update_reg(
                self.addr,
                Regs::GPPUB,
                (mask_set >> 8) as u8,
                (mask_clear >> 8) as u8,
            ).await?;
        }
        Ok(())
    }
}

impl<B: Mcp23x17Bus, RC, RM, ISR, IRQRC, IRQ> crate::PortDriverPolarity for Driver<B, RC, RM, ISR, IRQRC, IRQ>
where 
    RM: RawMutex,
    ISR: ISRPort,
    RC: Deref<Target = ISR> + Clone + AsRef<ISR> + Borrow<ISR>  
{
    async fn set_polarity(&mut self, mask: u32, polarity: crate::Polarity) -> Result<(), Self::Error> {
        let (mask_set, mask_clear) = match polarity {
            crate::Polarity::ActiveLow => (mask as u16, 0),
            crate::Polarity::ActiveHigh => (0, mask as u16),
        };
        if mask & 0x00FF != 0 {
            self.bus.update_reg(
                self.addr,
                Regs::IPOLA,
                (mask_set & 0xFF) as u8,
                (mask_clear & 0xFF) as u8,
            ).await?;
        }
        if mask & 0xFF00 != 0 {
            self.bus.update_reg(
                self.addr,
                Regs::IPOLB,
                (mask_set >> 8) as u8,
                (mask_clear >> 8) as u8,
            ).await?;
        }
        Ok(())
    }
}

impl<B: Mcp23x17Bus, ISRRC, RM, ISR, IRQ, IRQRC> crate::PortDriverInterrupts 
for Driver<B, ISRRC, RM, ISR, IRQRC, IRQ>
where 
    RM: RawMutex,
    ISR: ISRPort,
    IRQ: IRQPort,
    ISRRC: Deref<Target = ISR> + Clone + AsRef<ISR> + Borrow<ISR>,
    IRQRC: Deref<Target = IRQ> + Clone + AsRef<IRQ> + Borrow<IRQ>  
{
    async fn fetch_interrupt_state(&mut self) -> Result<(), Self::Error> {
        loop {
            self.isr.wait_for_interrupt().await;
            let inta = self.bus.read_reg(self.addr, Regs::INTCAPA).await?;
            let intb = self.bus.read_reg(self.addr, Regs::INTCAPB).await?;
            let state = ((intb as u16) << 8) | inta as u16;
            self.irq_port.push_irq((state as u32, self.out as u32));
            self.out = state;
        }
    }
}

impl<B: Mcp23x17Bus, ISRRC, RM, ISR, IRQ, IRQRC> crate::PortDriverIrqMask 
for Driver<B, ISRRC, RM, ISR, IRQRC, IRQ>
where 
    RM: RawMutex,
    ISR: ISRPort,
    IRQ: IRQPort,
    ISRRC: Deref<Target = ISR> + Clone + AsRef<ISR> + Borrow<ISR>,
    IRQRC: Deref<Target = IRQ> + Clone + AsRef<IRQ> + Borrow<IRQ>  
{
    async fn configure_interrupts(&mut self, mask_set: u32, mask_clear: u32, interrupt: crate::InterruptType) -> Result<(), Self::Error> {
        let (mask_set, mask_clear) = (mask_set as u16, mask_clear as u16);
        if mask_set & 0x00FF != 0 || mask_clear & 0x00FF != 0 {
            // make sure the relevant pins are set as inputs
            self.bus.update_reg(
                self.addr,
                Regs::IODIRA,
                (mask_set & 0xFF) as u8,
                (0) as u8,
            ).await?;
            match interrupt {
                crate::InterruptType::Falling | crate::InterruptType::Rising | 
                crate::InterruptType::Both => {
                    self.bus.update_reg(
                        self.addr, 
                        Regs::INTCONB, 
                        (mask_clear & 0xFF) as u8, 
                        (mask_set & 0xFF) as u8
                    ).await?;
                    self.bus.update_reg(
                        self.addr, 
                        Regs::DEFVALB, 
                        (mask_clear & 0xFF) as u8, 
                        (mask_set & 0xFF) as u8
                    ).await?;
                },
                crate::InterruptType::High => {
                    self.bus.update_reg(
                        self.addr, 
                        Regs::INTCONB, 
                        (mask_set & 0xFF) as u8, 
                        (mask_clear & 0xFF) as u8
                    ).await?;
                    self.bus.update_reg(
                        self.addr, 
                        Regs::DEFVALB, 
                        (mask_clear & 0xFF) as u8, 
                        (mask_set & 0xFF) as u8
                    ).await?;
                },
                crate::InterruptType::Low => {
                    self.bus.update_reg(
                        self.addr, 
                        Regs::INTCONB, 
                        (mask_set & 0xFF) as u8, 
                        (mask_clear & 0xFF) as u8
                    ).await?;
                    self.bus.update_reg(
                        self.addr, 
                        Regs::DEFVALB, 
                        (mask_set & 0xFF) as u8, 
                        (mask_clear & 0xFF) as u8
                    ).await?;
                },
            }
            self.bus.update_reg(
                self.addr, 
                Regs::GPINTENA, 
                (mask_set & 0xFF) as u8, 
                (mask_clear & 0xFF) as u8
            ).await?;
        }
        if mask_set & 0xFF00 != 0 || mask_clear & 0xFF00 != 0 {
            // make sure the relevant pins are set as inputs
            self.bus.update_reg(
                self.addr,
                Regs::IODIRB,
                (mask_set >> 8) as u8,
                0 as u8,
            ).await?;
            match interrupt {
                crate::InterruptType::Falling | crate::InterruptType::Rising | 
                crate::InterruptType::Both => {
                    self.bus.update_reg(
                        self.addr, 
                        Regs::INTCONB, 
                        (mask_clear >> 8) as u8, 
                        (mask_set >> 8) as u8
                    ).await?;
                    self.bus.update_reg(
                        self.addr, 
                        Regs::DEFVALB, 
                        (mask_clear >> 8) as u8, 
                        (mask_set >> 8) as u8
                    ).await?;
                },
                crate::InterruptType::High => {
                    self.bus.update_reg(
                        self.addr, 
                        Regs::INTCONB, 
                        (mask_set >> 8) as u8, 
                        (mask_clear >> 8) as u8
                    ).await?;
                    self.bus.update_reg(
                        self.addr, 
                        Regs::DEFVALB, 
                        (mask_clear >> 8) as u8, 
                        (mask_set >> 8) as u8
                    ).await?;
                },
                crate::InterruptType::Low => {
                    self.bus.update_reg(
                        self.addr, 
                        Regs::INTCONB, 
                        (mask_set >> 8) as u8, 
                        (mask_clear >> 8) as u8
                    ).await?;
                    self.bus.update_reg(
                        self.addr, 
                        Regs::DEFVALB, 
                        (mask_set >> 8) as u8, 
                        (mask_clear >> 8) as u8
                    ).await?;
                },
            }
            self.bus.update_reg(
                self.addr, 
                Regs::GPINTENB, 
                (mask_set >> 8) as u8, 
                (mask_clear >> 8) as u8
            ).await?;
        }
        Ok(())
    }
}

impl<B: Mcp23x17Bus, ISRRC, RM, ISR, IRQ, IRQRC> crate::PortDriverISR<IRQ, IRQRC> 
for Driver<B, ISRRC, RM, ISR, IRQRC, IRQ>
where 
    RM: RawMutex,
    ISR: ISRPort,
    IRQ: IRQPort,
    ISRRC: Deref<Target = ISR> + Clone + AsRef<ISR> + Borrow<ISR>,
    IRQRC: Deref<Target = IRQ> + Clone + AsRef<IRQ> + Borrow<IRQ>  
{
    fn get_isr(&mut self) -> IRQRC {
        self.irq_port.clone()
    }
}

// We need these newtype wrappers since we can't implement `Mcp23x17Bus` for both `I2cBus` and `SpiBus`
// at the same time
#[allow(missing_docs)]
pub struct Mcp23017Bus<I2C>(I2C);
#[allow(missing_docs)]
pub struct Mcp23S17Bus<SPI>(SPI);

/// Special -Bus trait for the Mcp23x17 since the SPI version is a bit special/weird in terms of writing
/// SPI registers, which can't necessarily be generialized for other devices.
#[allow(missing_docs)]
pub trait Mcp23x17Bus {
    type BusError;

    async fn write_reg<R: Into<u8>>(&mut self, addr: u8, reg: R, value: u8)
        -> Result<(), Self::BusError>;
    async fn read_reg<R: Into<u8>>(&mut self, addr: u8, reg: R) 
        -> Result<u8, Self::BusError>;
    async fn update_reg<R: Into<u8>>(
        &mut self,
        addr: u8,
        reg: R,
        mask_set: u8,
        mask_clear: u8,
    ) -> Result<(), Self::BusError> {
        let reg = reg.into();
        let mut val = self.read_reg(addr, reg).await?;
        val |= mask_set;
        val &= !mask_clear;
        self.write_reg(addr, reg, val).await?;
        Ok(())
    }
}

impl<SPI: crate::SpiBus> Mcp23x17Bus for Mcp23S17Bus<SPI> {
    type BusError = SPI::BusError;

    async fn write_reg<R: Into<u8>>(
        &mut self,
        addr: u8,
        reg: R,
        value: u8,
    ) -> Result<(), Self::BusError> {
        self.0.write(&[0x40 | addr << 1, reg.into(), value]).await?;

        Ok(())
    }

    async fn read_reg<R: Into<u8>>(&mut self, addr: u8, reg: R) -> Result<u8, Self::BusError> {
        let mut val = [0; 1];
        let write = [0x40 | addr << 1 | 0x1, reg.into()];
        let mut tx = [
            embedded_hal::spi::Operation::Write(&write),
            embedded_hal::spi::Operation::Read(&mut val),
        ];
        self.0.transaction(&mut tx).await?;

        Ok(val[0])
    }
}

impl<I2C: crate::I2cBus> Mcp23x17Bus for Mcp23017Bus<I2C> {
    type BusError = I2C::BusError;

    async fn write_reg<R: Into<u8>>(
        &mut self,
        addr: u8,
        reg: R,
        value: u8,
    ) -> Result<(), Self::BusError> {
        self.0.write_reg(addr, reg, value).await
    }

    async fn read_reg<R: Into<u8>>(&mut self, addr: u8, reg: R) -> Result<u8, Self::BusError> {
        self.0.read_reg(addr, reg).await
    }
}