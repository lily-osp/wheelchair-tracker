//! Bench probe #3: internet without PPP, through the module's own HTTP stack.
//!
//! The PPP path can be reset by carrier-side port-80 handling; `AT+HTTP*` runs
//! inside the module and is far easier to diagnose from the UART.
//!
//!   cargo run --release --bin http_probe
//!
//! Uses the `X-AIO-Key` query parameter because AT+HTTP cannot set custom
//! headers. Posts one point `{"value":"http-probe"}` to the location feed.

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::uart::{Config as UartConfig, Uart};
use heapless::String;

use firmware_rs::config as cfg;

esp_bootloader_esp_idf::esp_app_desc!();

const BAUD: u32 = 115_200;

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();
    let mut uart = Uart::new(p.UART1, UartConfig::default().with_baudrate(BAUD))
        .unwrap()
        .with_rx(p.GPIO5)
        .with_tx(p.GPIO4);

    esp_println::println!("http_probe: waiting for AT...");
    if !cmd(&mut uart, &delay, "AT", 1_000, &["OK"]) {
        esp_println::println!("no AT; run modem_probe first");
        idle(&delay);
    }
    cmd(&mut uart, &delay, "ATE0", 1_000, &["OK"]);
    cmd(&mut uart, &delay, "AT+CREG?", 2_000, &["OK"]);
    cmd(&mut uart, &delay, "AT+CGATT?", 2_000, &["OK"]);

    let mut url = String::<192>::new();
    let _ = core::fmt::Write::write_fmt(
        &mut url,
        format_args!(
            "AT+HTTPPARA=\"URL\",\"http://{}/api/v2/{}/feeds/{}/data?X-AIO-Key={}\"",
            cfg::SERVER_HOST, cfg::AIO_USER, cfg::AIO_FEED_LOCATION, cfg::AIO_KEY
        ),
    );

    if !cmd(&mut uart, &delay, "AT+HTTPINIT", 3_000, &["OK"]) {
        esp_println::println!("HTTPINIT failed on first try; terminating stale service and retrying");
        cmd(&mut uart, &delay, "AT+HTTPTERM", 3_000, &["OK"]);
        cmd(&mut uart, &delay, "AT+CGACT=1,1", 15_000, &["OK"]);
        cmd(&mut uart, &delay, "AT+CGPADDR=1", 3_000, &["OK"]);
        if !cmd(&mut uart, &delay, "AT+HTTPINIT", 5_000, &["OK"]) {
            esp_println::println!("HTTPINIT still failing");
            idle(&delay);
        }
    }
    cmd(&mut uart, &delay, "AT+HTTPPARA=\"CID\",1", 2_000, &["OK"]);
    cmd(&mut uart, &delay, url.as_str(), 3_000, &["OK"]);
    cmd(
        &mut uart,
        &delay,
        "AT+HTTPPARA=\"CONTENT\",\"application/json\"",
        2_000,
        &["OK"],
    );
    cmd(
        &mut uart,
        &delay,
        "AT+HTTPPARA=\"ACCEPT\",\"application/json\"",
        2_000,
        &["OK"],
    );

    let body = br#"{"value":"http-probe"}"#;
    let mut data_cmd = String::<48>::new();
    let _ = core::fmt::Write::write_fmt(
        &mut data_cmd,
        format_args!("AT+HTTPDATA={},20000", body.len()),
    );
    if !cmd(&mut uart, &delay, data_cmd.as_str(), 5_000, &["DOWNLOAD"]) {
        esp_println::println!("no DOWNLOAD prompt");
        idle(&delay);
    }
    let _ = uart.write(body);
    let _ = uart.write(b"\r\n");
    let _ = uart.flush();
    if !wait_for(&mut uart, &delay, 10_000, &["OK"]) {
        esp_println::println!("body was not accepted");
        idle(&delay);
    }
    esp_println::println!("body accepted, issuing AT+HTTPACTION=1");

    cmd(&mut uart, &delay, "AT+HTTPACTION=1", 2_000, &["OK"]);
    // URC: +HTTPACTION: 1,<status>,<len>
    if !wait_for(&mut uart, &delay, 60_000, &["+HTTPACTION:"]) {
        esp_println::println!("no +HTTPACTION within 60 s");
        cmd(&mut uart, &delay, "AT+HTTPTERM", 2_000, &["OK"]);
        idle(&delay);
    }

    cmd(&mut uart, &delay, "AT+HTTPREAD", 10_000, &["OK"]);
    cmd(&mut uart, &delay, "AT+HTTPTERM", 2_000, &["OK"]);
    esp_println::println!("done; check the feed for http-probe");

    idle(&delay)
}

fn idle(delay: &Delay) -> ! {
    loop {
        delay.delay_millis(5_000);
    }
}

/// Writes one command and waits for one of `needles`, printing every line.
fn cmd(
    uart: &mut Uart<'_, esp_hal::Blocking>,
    delay: &Delay,
    c: &str,
    wait_ms: u32,
    needles: &[&str],
) -> bool {
    esp_println::println!(">> {}", c);
    let _ = uart.write(c.as_bytes());
    let _ = uart.write(b"\r\n");
    let _ = uart.flush();
    wait_for(uart, delay, wait_ms, needles)
}

fn wait_for(
    uart: &mut Uart<'_, esp_hal::Blocking>,
    delay: &Delay,
    wait_ms: u32,
    needles: &[&str],
) -> bool {
    let mut line: String<160> = String::new();
    let mut hit = false;
    for _ in 0..(wait_ms / 10) {
        let mut buf = [0u8; 64];
        if let Ok(n) = uart.read_buffered(&mut buf) {
            for &b in &buf[..n] {
                match b {
                    b'\r' => {}
                    b'\n' => {
                        if !line.is_empty() {
                            esp_println::println!("   {}", line);
                            if needles.iter().any(|n| line.starts_with(n)) {
                                hit = true;
                            }
                            line.clear();
                        }
                    }
                    0x20..=0x7e => {
                        let _ = line.push(b as char);
                    }
                    _ => {}
                }
            }
        }
        if hit {
            // Keep draining briefly so the rest of the line/URC is visible.
            delay.delay_millis(50);
            return true;
        }
        delay.delay_millis(10);
    }
    if !line.is_empty() {
        esp_println::println!("   {}", line);
    }
    false
}
