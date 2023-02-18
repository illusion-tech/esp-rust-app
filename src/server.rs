use critical_section::with;
use embassy_time::{Duration, Timer};
use embedded_hal_1::digital::OutputPin;
use embedded_hal_nb::{serial::{Read, Write}, nb};
use fixedvec::{alloc_stack, FixedVec};
use log::{debug, error};
use rmodbus::{server::ModbusFrame, ModbusFrameBuf, ModbusProto};

use crate::CONTEXT;

pub struct Server<T, E> {
    id: u8,
    serial: Option<T>,
    rts: Option<E>,
}

impl<T: Read + Write, E: OutputPin> Server<T, E> {
    pub fn new(id: u8) -> Self {
        Server {
            id,
            serial: None,
            rts: None,
        }
    }

    pub async fn listen(&mut self, serial: T, rts: E) {
        self.serial = Some(serial);
        self.rts = Some(rts);

        let mut mem = fixedvec::alloc_stack!([u8; 256]);
        let mut ptr = 0;
        let mut idle = true;
        let mut delay = 0u8;

        loop {
            if idle {
                Timer::after(Duration::from_millis(100)).await;
            }

            while let Ok(byte) = self.serial.as_mut().expect("").read() {
                idle = false;
                debug!("{byte:02x}");
                mem[ptr] = byte;
                ptr += 1;
                delay = 0;
                Timer::after(Duration::from_micros(100)).await;
            }

            if idle {
                continue;
            }

            delay += 1;

            if delay > 10 {
                if ptr != 0 {
                    debug!("Received: {:2x?}, {}, {}", mem, delay, ptr);
                    self.proc_frame_buf(&mem).await;
                    mem.fill(0);
                    ptr = 0;
                }

                idle = true;
                delay = 0;
            }
        }
    }

    async fn proc_frame_buf(&mut self, buf: &ModbusFrameBuf) {
        let mut mem = alloc_stack!([u8; 256]);
        let mut response = FixedVec::new(&mut mem);
        let mut frame = ModbusFrame::new(self.id, buf, ModbusProto::Rtu, &mut response);

        if frame.parse().is_err() {
            error!("server error");
            return;
        }

        if frame.processing_required {
            debug!("processing required");
            with(|cs| {
                let mut context = CONTEXT.borrow_ref_mut(cs);
                let context = context.as_mut().unwrap();

                let result = match frame.readonly {
                    true => frame.process_read(context),
                    false => frame.process_write(context),
                };
                if result.is_err() {
                    error!("frame processing error");
                    // return;
                }
            });
        }

        if frame.response_required {
            debug!("response required");
            frame.finalize_response().unwrap();

            debug!("{:x?}", response);

            let rts = self.rts.as_mut().unwrap();

            rts.set_high().unwrap();
            Timer::after(Duration::from_micros(10)).await;

            response.as_slice().iter().for_each(|&byte| {
                self.serial.as_mut().unwrap().write(byte).unwrap();
            });

            nb::block!(self.serial.as_mut().unwrap().flush()).unwrap();

            rts.set_low().unwrap();
            debug!("Response {} bytes written", response.len());
        } else {
            debug!("no response required");
        }
    }
}
