#![no_std]
#![no_main]
#![feature(future_join)]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

extern crate alloc;

pub const ADDR_OFFSET: u32 = 0x100000;
pub const FLASH_SIZE: usize = 2 * 1024 * 1024;


use defmt::unwrap;
use embassy_executor::Executor;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::{bind_interrupts, peripherals, uart};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use defmt::info;
use multiplexer::{dev::mcp23x17, ExtIPin, Mcp23x17};
use embedded_alloc::TlsfHeap as Heap;
use rclite::Arc;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

#[global_allocator]
static HEAP: Heap = Heap::empty();
static mut CORE1_STACK: Stack<4096> = Stack::new();
static CORE0_EXEC: StaticCell<Executor> = StaticCell::new();
static CORE1_EXEC: StaticCell<Executor> = StaticCell::new();

bind_interrupts!(struct Irqs {
    UART0_IRQ => uart::InterruptHandler<peripherals::UART0>;
    I2C0_IRQ => i2c::InterruptHandler<peripherals::I2C0>;
});

// static MCP: StaticCell<
//     Mutex<
//         CriticalSectionRawMutex,
//         Mcp23x17<
//             Mutex<
//                 CriticalSectionRawMutex,
//                 mcp23x17::Driver<
//                     mcp23x17::Mcp23017Bus<I2c<peripherals::I2C0, i2c::Async>>,
//                     Arc<ExtIPin<Mutex<CriticalSectionRawMutex, Input>>>,
//                     CriticalSectionRawMutex,
//                     ExtIPin<Mutex<CriticalSectionRawMutex, Input>>,
//                     Arc<multiplexer::IrqPort<CriticalSectionRawMutex, 16>>,
//                 >,
//             >,
//         >,
//     >,
// > = StaticCell::new();

#[cortex_m_rt::entry]
fn main() -> ! {
    unsafe { embassy_rp::time_driver::init() }
    // Initialize the allocator BEFORE using it
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 4096;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let p = embassy_rp::init(Default::default());

    let _mcp_reset = Output::new(p.PIN_19, Level::High);
    let led = Output::new(p.PIN_25, Level::Low);

    let i2c_conf = embassy_rp::i2c::Config::default();
    let i2c_bus = I2c::new_async(p.I2C0, p.PIN_21, p.PIN_20, Irqs, i2c_conf);

    
    let exti_pin = Input::new(p.PIN_18, embassy_rp::gpio::Pull::None);
    let exti = Arc::new(ExtIPin(Mutex::<CriticalSectionRawMutex, _>::new(exti_pin)));
    let irq = Arc::new(multiplexer::IrqPort::<16>::new());

    let mcp: Mcp23x17<Mutex<_, mcp23x17::Driver<mcp23x17::Mcp23017Bus<I2c<'_, peripherals::I2C0, i2c::Async>>, Arc<ExtIPin<Mutex<CriticalSectionRawMutex, Input<'_>>>>, ExtIPin<Mutex<CriticalSectionRawMutex, Input<'_>>>, Arc<multiplexer::IrqPort<16>>>>> = Mcp23x17::new_mcp23017(i2c_bus, exti, irq, true, false, false);
    //let mcp = MCP.init(Mutex::new(mcp));

    // let drv_enable = block_on(async { mcp_pins.gpb6.into_output_high().await }).unwrap();

    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let core1_exec = CORE1_EXEC.init(Executor::new());
            core1_exec.run(|spawner| unwrap!(spawner.spawn(blink_task(spawner, led))));
        },
    );

    let executor0 = CORE0_EXEC.init(Executor::new());
    executor0.run(|spawner| unwrap!(spawner.spawn(core1_task(spawner, mcp))));
}

#[embassy_executor::task]
async fn blink_task(
    _spawner: Spawner,
    mut led: Output<'static>,
    
) {


    loop {
        info!("led 1 on!");
        led.set_high();
        Timer::after_secs(1).await;

        info!("led 1 off!");
        led.set_low();
        Timer::after_secs(1).await;
    }

}

#[embassy_executor::task]
async fn core1_task(_spawner: Spawner, mut mcp: Mcp23x17<
    Mutex<
        CriticalSectionRawMutex,
        mcp23x17::Driver<
            mcp23x17::Mcp23017Bus<I2c<'static, peripherals::I2C0, i2c::Async>>,
            Arc<ExtIPin<Mutex<CriticalSectionRawMutex, Input<'static>>>>,
            ExtIPin<Mutex<CriticalSectionRawMutex, Input<'static>>>,
            Arc<multiplexer::IrqPort<16>>,
        >,
    >,
>,) {
    Timer::after_secs(3).await;
    info!("core 1 task");
    let mcp_pins = mcp.split();
    
    let mut led2 = unwrap!(mcp_pins.gpb6.into_output().await);
    let mut led4 = unwrap!(mcp_pins.gpb7.into_output().await);

    loop {
        info!("led 2 on!");
        unwrap!(led2.set_low().await);
        unwrap!(led4.set_low().await);
        Timer::after_secs(1).await;

        info!("led 2 off!");
        unwrap!(led2.set_high().await);
        unwrap!(led4.set_high().await);
        Timer::after_secs(1).await;
    }
}
