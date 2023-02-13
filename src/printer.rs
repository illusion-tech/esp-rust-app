#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        use crate::printer::Printer;
        writeln!(Printer, $($arg)*).ok();
    }};
}

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        use crate::printer::Printer;
        write!(Printer, $($arg)*).ok();
    }};
}

const UART_TX_ONE_CHAR: usize = 0x4000_0068;
pub struct Printer;

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
