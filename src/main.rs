#![no_std]
#![no_main]
mod printer;

use core::{cell::RefCell, fmt::Write};
use critical_section::Mutex;
use esp32c3_hal::{
    clock::ClockControl,
    gpio::{Event, Gpio10, Gpio9, Input, Output, PullDown, PushPull, IO},
    interrupt::{self, CpuInterrupt, InterruptKind, Priority},
    peripherals::{Interrupt, Peripherals, UART1},
    prelude::*,
    riscv,
    timer::TimerGroup,
    uart::{
        config::{Config, DataBits, Parity, StopBits},
        TxRxPins,
    },
    Cpu, Delay, Rtc, Uart,
};
use esp_backtrace as _;

static BUTTON: Mutex<RefCell<Option<Gpio9<Input<PullDown>>>>> = Mutex::new(RefCell::new(None));
static GPIO10: Mutex<RefCell<Option<Gpio10<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static SERIAL1: Mutex<RefCell<Option<Uart<UART1>>>> = Mutex::new(RefCell::new(None));

#[entry]
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

    // 设置 GPIO9（板载 boot 按钮）为输入
    let mut button = io.pins.gpio9.into_pull_down_input();
    // 设置 GPIO10 为输出控制 MAX3485 收发状态
    let mut gpio10 = io.pins.gpio10.into_push_pull_output();
    // 设置 UART1 控制器，并使用了 GPIO0 和 GPIO1 作为 TX 和 RX
    let mut serial1 = Uart::new_with_config(
        peripherals.UART1,
        Some(Config {
            baudrate: 115_200,
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
    gpio10.set_low().unwrap();
    // gpio10.set_
    serial1.set_rx_fifo_full_threshold(5);
    serial1.listen_rx_fifo_full();

    critical_section::with(|cs| BUTTON.borrow_ref_mut(cs).replace(button));
    critical_section::with(|cs| GPIO10.borrow_ref_mut(cs).replace(gpio10));
    critical_section::with(|cs| SERIAL1.borrow_ref_mut(cs).replace(serial1));

    interrupt::enable(Interrupt::GPIO, Priority::Priority3).unwrap();
    interrupt::enable(Interrupt::UART1, Priority::Priority1).unwrap();
    interrupt::set_kind(Cpu::ProCpu, CpuInterrupt::Interrupt1, InterruptKind::Edge);

    unsafe {
        riscv::interrupt::enable();
    }

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
        let mut gpio10 = GPIO10.borrow_ref_mut(cs);
        let gpio10 = gpio10.as_mut().unwrap();

        // 设置高电平输出使 MAX3485 进入发送状态
        gpio10.set_high().unwrap();

        // 通过串口发送数据
        // FIXME: 无法完整发送数据
        serial1.write_bytes("Hello World!".as_bytes()).ok();
        // writeln!(serial1, "World!").ok();

        // 设置低电平使 MAX3485 恢复接受状态
        gpio10.set_low().unwrap();

        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();
    });
}

#[interrupt]
fn UART1() {
    critical_section::with(|cs| {
        let mut serial1 = SERIAL1.borrow_ref_mut(cs);
        let serial1 = serial1.as_mut().unwrap();

        // 从串口中读取数据直至缓冲区为空
        while let Ok(byte) = serial1.read() {
            print!("{}", byte as char);
        }

        serial1.reset_at_cmd_interrupt();
        serial1.reset_rx_fifo_full_interrupt();
    });
}
