#![no_std]
#![no_main]

use esp32c3_hal::{
    clock::ClockControl, peripherals::Peripherals, prelude::*, timer::TimerGroup, Rtc,
};
use esp_backtrace as _;
use log::{set_logger_racy, set_max_level, LevelFilter, Log, Metadata, Record};

const UART_TX_ONE_CHAR: usize = 0x4000_0068;
struct Printer;

impl Printer {
    fn write_bytes(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            unsafe {
                type T = unsafe extern "C" fn(u8) -> i32;
                core::mem::transmute::<usize, T>(UART_TX_ONE_CHAR)(byte);
            }
        }
    }
}

impl core::fmt::Write for Printer {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        Printer.write_bytes(s.as_bytes());
        Ok(())
    }
}
macro_rules! println {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        writeln!(Printer, $($arg)*).ok();
    }};
}

struct EspLogger;

impl Log for EspLogger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        println!("{} - {}", record.level(), record.args());
    }

    fn flush(&self) {
        todo!()
    }
}

fn init_logger(level: LevelFilter) {
    unsafe {
        set_logger_racy(&EspLogger).unwrap();
        set_max_level(level);
    }
}

#[riscv_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    loop {}
}
