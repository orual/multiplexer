#![doc = include_str!("../README.md")]
#![warn(missing_docs)]
#![allow(dead_code)]
#![cfg_attr(not(any(test, feature = "std")), no_std)]
#![cfg_attr(nightly, allow(stable_features, unknown_lints))]
#![cfg_attr(nightly, feature(async_fn_in_trait, impl_trait_projections))]
#![allow(async_fn_in_trait, impl_trait_projections)]
#[allow(unused_imports)]


mod bus;
mod common;
pub mod dev;
mod mutex;
mod pin;
mod isr;
mod yield_on;


pub use bus::I2cBus;
pub use common::mode;
pub use common::InterruptType;
pub use pin::Pin;

pub(crate) use bus::I2cExt;
pub(crate) use bus::SpiBus;
pub(crate) use common::Direction;
#[allow(unused_imports)]
pub(crate) use common::Drive;
pub(crate) use common::Polarity;
pub(crate) use common::PortDriver;
pub(crate) use isr::IRQPort;
#[allow(unused_imports)]
pub(crate) use isr::ISRPort;
pub(crate) use common::PortDriverPolarity;
pub(crate) use common::PortDriverPullDown;
pub(crate) use common::PortDriverPullUp;
pub(crate) use common::PortDriverDirection;
#[allow(unused_imports)]
pub(crate) use common::PortDriverOutputDrive;
pub(crate) use common::PortDriverInterrupts;
pub(crate) use common::PortDriverIrqMask;
pub(crate) use common::PortDriverISR;
#[allow(unused_imports)]
pub(crate) use common::PortDriverExtI;


pub use dev::mcp23x17::Mcp23x17;
pub use dev::mcp23x17::McpExtIPins;
pub use isr::IrqPort;
pub use isr::ExtIPin;