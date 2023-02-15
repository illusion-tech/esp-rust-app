#![no_std]
#![no_main]
mod printer;

use core::{cell::RefCell, fmt::Write};
// use ;
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
        config::{AtCmdConfig, Config, DataBits, Parity, StopBits},
        TxRxPins,
    },
    Cpu, Delay, Rtc, Uart,
};
use esp_backtrace as _;

// use rmodbus

static DELAY: Mutex<RefCell<Option<Delay>>> = Mutex::new(RefCell::new(None));

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
            parity: Parity::ParityEven,
            stop_bits: StopBits::STOP1,
        }),
        Some(TxRxPins::new_tx_rx(
            io.pins.gpio0.into_push_pull_output(),
            io.pins.gpio1.into_pull_up_input(),
        )),
        &clocks,
    );

    button.listen(Event::FallingEdge);
    // #[cfg(feature = "server")]
    // {
    gpio10.set_low().unwrap();
    // }
    // #[cfg(feature = "client")]
    // {
    //     gpio10.set_high().unwrap();
    // }

    serial1.set_at_cmd(AtCmdConfig::new(None, None, None, b'#', None));
    serial1.set_rx_fifo_full_threshold(64);
    serial1.listen_at_cmd();
    serial1.listen_rx_fifo_full();
    serial1.listen_tx_done();

    critical_section::with(|cs| BUTTON.borrow_ref_mut(cs).replace(button));
    critical_section::with(|cs| GPIO10.borrow_ref_mut(cs).replace(gpio10));
    critical_section::with(|cs| SERIAL1.borrow_ref_mut(cs).replace(serial1));

    // #[cfg(feature = "server")]
    // {
    interrupt::enable(Interrupt::GPIO, Priority::Priority3).unwrap();
    // }
    interrupt::enable(Interrupt::UART1, Priority::Priority1).unwrap();
    interrupt::set_kind(Cpu::ProCpu, CpuInterrupt::Interrupt1, InterruptKind::Edge);

    unsafe {
        riscv::interrupt::enable();
    }

    let delay = Delay::new(&clocks);

    critical_section::with(|cs| DELAY.borrow_ref_mut(cs).replace(delay));

    loop {
        // critical_section::with(|cs| {
        //     let mut delay = DELAY.borrow_ref_mut(cs);
        //     let delay = delay.as_mut().unwrap();
        // });
        Delay::new(&clocks).delay_ms(500u32);
    }
}

#[interrupt]
fn GPIO() {
    critical_section::with(|cs| {
        println!("GPIO 中断");

        let mut serial1 = SERIAL1.borrow_ref_mut(cs);
        let serial1 = serial1.as_mut().unwrap();
        let mut gpio10 = GPIO10.borrow_ref_mut(cs);
        let gpio10 = gpio10.as_mut().unwrap();

        // 设置高电平输出使 MAX3485 进入发送状态
        gpio10.set_high().unwrap();

        // 通过串口发送数据
        // write!(serial1, "#Hello World!").ok();

        let mut mdb_req = rmodbus::client::ModbusRequest::new(1, rmodbus::ModbusProto::Rtu);
        let mut space = fixedvec::alloc_stack!([u8; 256]);
        let mut request = fixedvec::FixedVec::new(&mut space);
        mdb_req
            .generate_set_holdings_string(
                0,
                "Hello World from ESP32C3 using Rust over RS483 using Modbus protocol!\r\n",
                &mut request,
            )
            .unwrap();

        print!("{:?}", request);
        serial1.write_bytes(request.as_slice()).ok();

        let mut delay = DELAY.borrow_ref_mut(cs);
        let delay = delay.as_mut().unwrap();

        nb::block!(serial1.flush()).ok();
        delay.delay_ms(request.len() as u32);

        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();

        // 设置低电平使 MAX3485 恢复接受状态
        gpio10.set_low().unwrap();
    });
}

#[interrupt]
fn UART1() {
    critical_section::with(|cs| {
        println!("\r\nUART1 中断");

        let mut serial1 = SERIAL1.borrow_ref_mut(cs);
        let serial1 = serial1.as_mut().unwrap();

        if serial1.tx_done_interrupt_set() {
            println!("TX-DONE 中断");
            serial1.reset_tx_done_interrupt();

            return;
        }

        println!(
            "中断类型 AT-CMD: {} RX-FIFO-FULL: {}",
            serial1.at_cmd_interrupt_set(),
            serial1.rx_fifo_full_interrupt_set()
        );

        // let mut buf: rmodbus::ModbusFrameBuf = [0; 256];

        // serial1

        // 从串口中读取数据直至缓冲区为空
        let mut cnt = 0;
        // let x = nb::block!(serial1.read());
        while let Ok(_byte) = nb::block!(serial1.read()) {
            cnt += 1;
            print!("{}", _byte as char);
        }

        println!("\r\n读取 {} 字节", cnt);

        serial1.reset_at_cmd_interrupt();
        serial1.reset_rx_fifo_full_interrupt();
    });
}
