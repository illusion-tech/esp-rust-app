#![no_std]
#![no_main]
mod printer;
use esp32c3_hal::{
    clock::ClockControl, peripherals::Peripherals, prelude::*, pulse_control::RepeatMode,
    timer::TimerGroup, Delay, Rtc, IO,
};
use esp_backtrace as _;

#[riscv_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
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

    println!("Hello, world!");

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let button = io.pins.gpio9.into_pull_up_input();
    let mut delay = Delay::new(&clocks);

    // let b = system.;
    let pulse = esp32c3_hal::PulseControl::new(
        peripherals.RMT,
        &mut system.peripheral_clock_control,
        esp32c3_hal::pulse_control::ClockSource::APB,
        0,
        0,
        0,
    )
    .unwrap();

    let mut channel = pulse.channel0;

    channel
        .set_idle_output_level(false)
        .set_carrier_modulation(false)
        .set_channel_divider(1)
        .set_idle_output(true);

    let mut led = channel.assign_pin(io.pins.gpio8);

    // let mut rmt_buffer = [0u32; 25];
    // rmt_buffer

    match led.send_pulse_sequence_raw(RepeatMode::SingleShot, &[0u32; 25]) {
        Ok(_) => println!("Success!"),
        Err(e) => println!("Error: {:?}", e),
    }

    loop {
        if button.is_high().unwrap() {
        } else {
            println!("Button is pressed!");
            delay.delay_ms(500u32);
        }
    }
}
