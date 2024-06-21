#![cfg_attr(not(any(test, feature = "std")), no_std)]

mod bus;
pub use bus::I2cBus;


pub(crate) use bus::I2cExt;
pub(crate) use bus::SpiBus;