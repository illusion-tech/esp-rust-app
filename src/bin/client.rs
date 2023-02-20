#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::{task, Executor};
use embassy_time::{Duration, Timer};
use embedded_hal_async::digital::Wait;
use embedded_hal_nb::nb::block;
use esp32c3_hal::{
    clock::ClockControl,
    embassy, entry,
    gpio::{Gpio10, Gpio9, Input, Output, PullDown, PushPull},
    interrupt::{self, Priority},
    peripherals::{Interrupt, Peripherals, UART1},
    prelude::*,
    systimer::SystemTimer,
    timer::TimerGroup,
    uart::{config::Config, TxRxPins},
    Rtc, Uart, IO,
};
use esp_backtrace as _;
use esp_println::logger::init_logger;
use log::{debug, LevelFilter};
use rmodbus::{client::ModbusRequest, ModbusProto};
use static_cell::StaticCell;

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[task]
async fn run() {
    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn run_client(
    mut button: Gpio9<Input<PullDown>>,
    mut serial: Uart<'static, UART1>,
    mut rts: Gpio10<Output<PushPull>>,
) {
    loop {
        button.wait_for_rising_edge().await.unwrap();

        let mut client = ModbusRequest::new(1, ModbusProto::Rtu);

        let mut mem = fixedvec::alloc_stack!([u8; 256]);
        let mut request = fixedvec::FixedVec::new(&mut mem);
        client.generate_set_coil(100, true, &mut request).unwrap();

        debug!("Request: {:x?}", request);

        rts.set_high().unwrap();
        // Timer::after(Duration::from_micros(10)).await;
        serial.write_bytes(request.as_slice()).unwrap();
        block!(serial.flush()).unwrap();
        rts.set_low().unwrap();
        debug!("Request {} bytes written", request.len());

    }
}

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
    let button = io.pins.gpio9.into_pull_down_input();
    let mut rts = io.pins.gpio10.into_push_pull_output();

    let config = Config::default();
    let pins = TxRxPins::new_tx_rx(
        io.pins.gpio0.into_push_pull_output(),
        io.pins.gpio1.into_floating_input(),
    );
    let serial1 = Uart::new_with_config(peripherals.UART1, Some(config), Some(pins), &clocks);

    rts.set_low().unwrap();
    interrupt::enable(Interrupt::GPIO, Priority::Priority3).unwrap();

    let executor = EXECUTOR.init(Executor::new());

    executor.run(|spawner| {
        spawner.spawn(run()).unwrap();
        spawner.spawn(run_client(button, serial1, rts)).unwrap();
    });
}
