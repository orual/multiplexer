# `multiplexer`

This crate provides a common set of abstractions for I2C and SPI GPIO expander chips. There is a performance overhead (both that inherent to a serial data link and to the synchronization primitives used internally), but the architecture allows implementation of `embedded-hal` and `embedded-hal-async` digital IO traits for each individual pin, and for pins to be shared individually like any other GPIO pin, making it a relatively transparent abstraction and enabling integration with the many drivers that target those traits. In particular, this crate supports asynchronous individual pin-level interrupts by implementing the `embedded_hal_async::digital::Wait` trait for each GPIO pin.

## Example

```rust
// This example uses reference-counted pointers from alloc.
// It should also support non-alloc equivalents from the heapless crate.
extern crate alloc;
use alloc::rc::Rc;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embedded_hal::digital::OutputPin;


// Initialize I2C peripheral from HAL
let i2c = todo!();
// Initialize ISR pins via HAL methods,
let (ia, ib) = todo!()

// Set up external interrupt systems
// Some of this housekeeping will be cleaned up in future updates.
let (ia, ib) = (multiplexer::ExtIPin(ia), multiplexer::ExtIPin(ib));
let exti = Rc::new(multiplexer::McpExtIPins(ia, ib));
let irq = Rc::new(multiplexer::IrqPort::<NoopRawMutex, 16>::new());

// A0: HIGH, A1: LOW, A2: LOW
let mut mcp = multiplexer::Mcp23x17::new(i2c, exti, irq, true, false, false);
let mcp_pins = mcp.split();

async {
    let pin0 = mcp_pins.gpa0.into_output().await.unwrap();
    let pin3 = mcp_pins.gpa3.into_isr_pin().await.unwrap(); // Pins don't respond to interrupts by default

    
    <pin5 as OutputPin>.set_high().unwrap() // HAL trait method (blocking)
    assert!(io1_5.is_high().unwrap());
    pin3.wait_for_rising_edge().await; // waits here, sleeping if this is running on Embassy
    pin5.set_low().await.unwrap() // built-in pin method (async)
    assert!(io1_5.is_low().unwrap());
}
```

### Origin

This crate is heavily based on the [`port-expander`](https://github.com/rahix/port-expander) crate. I have quite a debt to their work here. However, porting the underlying bus wrappers to the `embedded-hal-async` SPI and I2C traits (for power efficiency reasons), as well as the implementation of pin interrupts (currently under discussion by the maintainers, but unimplemented, and with strictly blocking/polling forms planned) and the `Wait` trait, necessitated substantial code changes which made a "fork and pull request" strategy undesirable.

One substantial architectural change was the switch from the custom PortMutex trait to the `embassy-sync` Mutex implementations. I needed an async-compatible mutex (which PortMutex was not), didn't want to reinvent the wheel, and the Embassy library crates are popular. `embassy-sync` offers a variety of underlying Mutex implementations, which this crate is generic over.

## Supported Devices

Currently, only the MCP23x17 (both I2C and SPI variants) is fully supported. Work on porting the other IO-expanders supported by `port-expander` is ongoing but the MCP was the specific device I personally needed to support this behaviour.