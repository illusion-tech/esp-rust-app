//! GPIO interrupt
//!
//! This prints "Interrupt" when the boot button is pressed.

#![no_std]
#![no_main]
mod printer;
use core::{cell::RefCell, fmt::Write};
use critical_section::Mutex;
use esp32c3_hal::uart::TxRxPins;
use esp32c3_hal::{
    clock::ClockControl,
    gpio::{Event, Gpio9, Input, PullDown, IO},
    interrupt,
    peripherals::{self, Peripherals, UART1},
    prelude::*,
    timer::TimerGroup,
    uart::config::{Config, DataBits, Parity, StopBits},
    Cpu,
    Delay,
    Rtc,
    Uart,
};
use esp_backtrace as _;
use riscv_rt;

static BUTTON: Mutex<RefCell<Option<Gpio9<Input<PullDown>>>>> = Mutex::new(RefCell::new(None));
static SERIAL1: Mutex<RefCell<Option<Uart<UART1>>>> = Mutex::new(RefCell::new(None));

#[riscv_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Set GPIO9 as an input
    let mut button = io.pins.gpio9.into_pull_down_input();
    // let mut serial1 = Uart::new(peripherals.UART1);
    let serial1 = Uart::new_with_config(
        peripherals.UART1,
        Some(Config {
            baudrate: 115200,
            data_bits: DataBits::DataBits8,
            parity: Parity::ParityNone,
            stop_bits: StopBits::STOP1,
        }),
        Some(TxRxPins::new_tx_rx(
            io.pins.gpio0.into_push_pull_output(),
            io.pins.gpio1.into_floating_input(),
        )),
        &clocks,
    );

    button.listen(Event::FallingEdge);

    // serial1.set_rx_fifo_full_threshold(30);
    // serial1.listen_rx_fifo_full();

    critical_section::with(|cs| BUTTON.borrow_ref_mut(cs).replace(button));
    critical_section::with(|cs| SERIAL1.borrow_ref_mut(cs).replace(serial1));

    interrupt::enable(peripherals::Interrupt::GPIO, interrupt::Priority::Priority3).unwrap();
    interrupt::enable(peripherals::Interrupt::UART1, interrupt::Priority::Priority1).unwrap();
    interrupt::set_kind(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt1,
        interrupt::InterruptKind::Edge,
    );

    let mut delay = Delay::new(&clocks);
    println!("Hello world!");
    loop {
        print!(".");
        // led.toggle().unwrap();
        delay.delay_ms(500u32);
    }
}

#[interrupt]
fn GPIO() {
    critical_section::with(|cs| {
        println!("GPIO interrupt");

        let mut serial1 = SERIAL1.borrow_ref_mut(cs);
        let serial1 = serial1.as_mut().unwrap();

        // let serial1 = SERIAL1.borrow_ref_mut(cs).as_mut().unwrap();

        writeln!(serial1, "Hello World!").ok();

        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();
    });
}
