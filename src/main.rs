//! embassy hello world
//!
//! This is an example of running the embassy executor with multiple tasks
//! concurrently.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
mod printer;
use embassy_executor::Executor;
use embassy_time::{Duration, Timer};
use embedded_hal_async::digital::Wait;
use esp32c3_hal::{
    embassy,
    gpio::{Floating, Gpio10, Gpio9, Input, Output, PullDown, PushPull},
    peripherals::UART1,
    prelude::*,
    Uart,
};
use esp_backtrace as _;
use static_cell::StaticCell;

#[embassy_executor::task]
async fn run1() {
    loop {
        println!("Hello world from embassy using esp-hal-async!");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[embassy_executor::task]
async fn run2() {
    #[cfg(feature = "server")]
    loop {
        println!("Bing!");
        Timer::after(Duration::from_millis(5_000)).await;
    }
}

#[embassy_executor::task]
async fn run3(
    mut button: Gpio9<Input<PullDown>>,
    mut serial: Uart<'static, UART1>,
    mut rts: Gpio10<Output<PushPull>>,
) {
    #[cfg(feature = "server")]
    let mut mem = fixedvec::alloc_stack!([u8; 256]);
    #[cfg(feature = "server")]
    loop {
        // let byte = serial.read().unwrap();
        // let r = );
        let mut ticks = 0u8;
        let mut idx = 0;
        loop {
            while let Ok(byte) = serial.read() {
                println!("{byte:02x}");
                mem[idx] = byte;
                idx += 1;
                ticks = 0;
                Timer::after(Duration::from_micros(100)).await;
            }

            if idx != 0 {
                ticks += 1;
            }

            if ticks > 10 {
                if idx != 0 {
                    break;
                }
                
                ticks = 0;
            }
        }

        println!("Received: {:?}, {}, {}", mem, ticks, idx);

        mem.fill(0);
        idx = 0;
    }

    // match serial.read() {
    //     Ok(byte) => {
    //         println!("{byte:02x}");
    //         ticks = 0;
    //     }
    //     Err(e) => {
    //         ticks += 1;
    //         println!("Error: {:?}", e);
    //     }
    // }
    // print!("{byte:02x} ");
    // }
    #[cfg(not(feature = "server"))]
    loop {
        button.wait_for_rising_edge().await.unwrap();
        println!("Ping!");

        let mut client = rmodbus::client::ModbusRequest::new(1, rmodbus::ModbusProto::Rtu);

        let mut mem = fixedvec::alloc_stack!([u8; 256]);
        let mut request = fixedvec::FixedVec::new(&mut mem);
        client.generate_set_coil(100, true, &mut request).unwrap();

        println!("Request: {:?}", request);

        rts.set_high().unwrap();

        Timer::after(Duration::from_micros(10)).await;
        serial.write_bytes(request.as_slice()).unwrap();
        println!("{} bytes written", request.len());
        // serial.write(b'.').unwrap();

        // let d = <usize as TryInto<u64>>::try_into(request.len()).unwrap() * 2;
        // Timer::after(Duration::from_micros(d)).await;
        rts.set_low().unwrap();
    }
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    println!("Init!");
    let peripherals = esp32c3_hal::peripherals::Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = esp32c3_hal::clock::ClockControl::boot_defaults(system.clock_control).freeze();
    let sys_timer = esp32c3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER);

    embassy::init(&clocks, sys_timer);

    let mut rtc = esp32c3_hal::Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = esp32c3_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = esp32c3_hal::timer::TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = esp32c3_hal::IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let button = io.pins.gpio9.into_pull_down_input();
    let mut rts = io.pins.gpio10.into_push_pull_output();
    let config = esp32c3_hal::uart::config::Config {
        baudrate: 115_200,
        data_bits: esp32c3_hal::uart::config::DataBits::DataBits8,
        parity: esp32c3_hal::uart::config::Parity::ParityEven,
        stop_bits: esp32c3_hal::uart::config::StopBits::STOP1,
    };
    // let pins = esp32c3_hal::uart::AllPins::<_, _, Gpio9<Input<Floating>>, _> {
    //     tx: Some(io.pins.gpio0.into_push_pull_output().into_ref()),
    //     rx: Some(io.pins.gpio1.into_floating_input().into_ref()),
    //     rts: Some(io.pins.gpio10.into_push_pull_output().into_ref()),
    //     cts: None,
    // };
    let pins = esp32c3_hal::uart::TxRxPins::new_tx_rx(
        io.pins.gpio0.into_push_pull_output(),
        io.pins.gpio1.into_floating_input(),
    );
    let serial1 =
        esp32c3_hal::Uart::new_with_config(peripherals.UART1, Some(config), Some(pins), &clocks);

    rts.set_low().unwrap();
    esp32c3_hal::interrupt::enable(
        esp32c3_hal::peripherals::Interrupt::GPIO,
        esp32c3_hal::interrupt::Priority::Priority3,
    )
    .unwrap();

    let executor = EXECUTOR.init(Executor::new());

    executor.run(|spawner| {
        spawner.spawn(run1()).ok();
        spawner.spawn(run2()).ok();
        spawner.spawn(run3(button, serial1, rts)).ok();

        // serial1.read();
    });
}
