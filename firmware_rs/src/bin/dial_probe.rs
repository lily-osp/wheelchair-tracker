//! Temporary bench tool: dump the raw `ATD*99***1#` exchange.
#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::uart::{Config as UartConfig, Uart};

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();
    let mut uart = Uart::new(p.UART1, UartConfig::default().with_baudrate(115_200))
        .unwrap()
        .with_rx(p.GPIO5)
        .with_tx(p.GPIO4);

    drain(&mut uart, &delay, 300);
    cmd(&mut uart, &delay, "AT", 600, true);
    cmd(&mut uart, &delay, "ATE0", 400, true);
    cmd(&mut uart, &delay, "AT+CGDCONT=1,\"IP\",\"internet\"", 600, true);
    cmd(&mut uart, &delay, "AT+CGACT?", 600, true);
    cmd(&mut uart, &delay, "AT+CGACT=1,1", 2_000, true);
    cmd(&mut uart, &delay, "AT+CGPADDR=1", 800, true);

    esp_println::println!("\n--- ATD*99***1# ---");
    let _ = uart.write(b"ATD*99***1#\r\n");
    let _ = uart.flush();
    raw(&mut uart, &delay, 9_000);

    esp_println::println!("\n--- escape ---");
    delay.delay_millis(1_200);
    let _ = uart.write(b"+++");
    let _ = uart.flush();
    delay.delay_millis(1_200);
    cmd(&mut uart, &delay, "ATH", 2_000, true);

    loop {
        delay.delay_millis(5_000);
    }
}

fn cmd(uart: &mut Uart<'_, esp_hal::Blocking>, delay: &Delay, c: &str, wait: u32, echo: bool) {
    let _ = uart.write(c.as_bytes());
    let _ = uart.write(b"\r\n");
    let _ = uart.flush();
    if echo {
        esp_println::println!(">> {}", c);
    }
    raw(uart, delay, wait);
}

/// Prints the reply as printable text, marking anything non-ASCII.
fn raw(uart: &mut Uart<'_, esp_hal::Blocking>, delay: &Delay, wait_ms: u32) {
    let mut line = heapless::String::<160>::new();
    for _ in 0..(wait_ms / 10) {
        let mut buf = [0u8; 64];
        if let Ok(n) = uart.read_buffered(&mut buf) {
            for &b in &buf[..n] {
                if b == b'\r' {
                    continue;
                }
                if b == b'\n' {
                    if !line.is_empty() {
                        esp_println::println!("  {}", line);
                        line.clear();
                    }
                    continue;
                }
                if (0x20..0x7f).contains(&b) {
                    let _ = line.push(b as char);
                } else {
                    let _ = core::fmt::Write::write_fmt(
                        &mut line,
                        format_args!("\\x{:02x}", b),
                    );
                }
            }
        }
        delay.delay_millis(10);
    }
    if !line.is_empty() {
        esp_println::println!("  {}", line);
    }
}

fn drain(uart: &mut Uart<'_, esp_hal::Blocking>, delay: &Delay, wait_ms: u32) {
    raw(uart, delay, wait_ms);
}
