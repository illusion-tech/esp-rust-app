#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;

use critical_section::{with, Mutex};
use embassy_executor::{task, Executor};
use embassy_time::{Duration, Timer};
// use embedded_hal_async::digital::Wait;
use esp32c3_hal::{
    clock::ClockControl,
    embassy, entry,
    interrupt::{self, Priority},
    peripherals::{Interrupt, Peripherals},
    prelude::*,
    systimer::SystemTimer,
    timer::TimerGroup,
    uart::{config::Config, TxRxPins},
    Rtc, Uart, IO,
};
use esp_backtrace as _;
use esp_println::logger::init_logger;
use log::LevelFilter;
use rmodbus::server::context::ModbusContext;
use static_cell::StaticCell;

static EXECUTOR: StaticCell<Executor> = StaticCell::new();
static CONTEXT: Mutex<RefCell<Option<ModbusContext>>> = Mutex::new(RefCell::new(None));

#[task]
async fn run() {
    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

// #[task]
// async fn server {}

#[entry]
fn main() -> ! {
    init_logger(LevelFilter::Debug);

    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let sys_timer = SystemTimer::new(peripherals.SYSTIMER);

    embassy::init(&clocks, sys_timer);

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut rts = io.pins.gpio10.into_push_pull_output();

    let config = Config::default();
    let pins = TxRxPins::new_tx_rx(
        io.pins.gpio0.into_push_pull_output(),
        io.pins.gpio1.into_floating_input(),
    );
    let serial1 = Uart::new_with_config(peripherals.UART1, Some(config), Some(pins), &clocks);

    rts.set_low().unwrap();
    interrupt::enable(Interrupt::GPIO, Priority::Priority3).unwrap();

    with(|cs| CONTEXT.replace(cs, Some(ModbusContext::new())));

    let executor = EXECUTOR.init(Executor::new());

    executor.run(|spawner| {
        spawner.spawn(run()).unwrap();
    });
}
