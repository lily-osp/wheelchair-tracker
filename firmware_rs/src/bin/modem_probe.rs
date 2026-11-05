//! Bench probe: proves the ESP32-C3 reaches the SIM7600 over UART1.
//!
//! No SIM and no antenna are needed. `AT`, `ATI` and `AT+CGMR` answering at
//! all is the pass condition; `+CME ERROR: 10` from `AT+CPIN?` is the correct
//! reply with an empty slot, and `+CSQ: 99,99` / `+CREG: 0,0` are expected
//! with no antenna and no card.
//!
//! Wiring (HAT UART jumper on B, VCCIO 3.3 V, common ground):
//!   C3 GPIO4 (TX) -> HAT RXD
//!   C3 GPIO5 (RX) -> HAT TXD
//!
//! Flash and watch:
//!   cargo run --release --bin modem_probe

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::uart::{Config as UartConfig, Uart};
use heapless::String;

esp_bootloader_esp_idf::esp_app_desc!();

const BAUD: u32 = 115_200;
/// The module takes ~15 s from power-on to first `AT`; be generous.
const BOOT_PROBE_MS: u32 = 45_000;

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();

    // Held low and never pulsed: the HAT auto-powers from PWR-3V3.
    let _pwrkey = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());

    let mut uart = Uart::new(
        peripherals.UART1,
        UartConfig::default().with_baudrate(BAUD),
    )
    .unwrap()
    .with_rx(peripherals.GPIO5)
    .with_tx(peripherals.GPIO4);

    esp_println::println!(
        "\nmodem_probe: UART1 {} 8N1, TX=GPIO4 RX=GPIO5\nwaiting for the module...",
        BAUD
    );

    let mut alive = false;
    for i in 0..(BOOT_PROBE_MS / 500) {
        if send(&mut uart, &delay, "AT", 400) {
            alive = true;
            esp_println::println!("module answered AT after ~{} ms", (i + 1) * 500);
            break;
        }
        esp_println::print!(".");
    }

    if !alive {
        esp_println::println!(
            "\nno response.\n\
             check, in order:\n\
             1. HAT UART jumper is on B (module <-> host header), not A or C\n\
             2. RX/TX are crossed: C3 TX GPIO4 -> HAT RXD, C3 RX GPIO5 -> HAT TXD\n\
             3. VCCIO is 3.3 V and the HAT PWR LED is on (PWR-3V3 fitted)\n\
             4. grounds are shared, 5 V reaches the HAT\n\
             5. baud: try 9600 if the module was reconfigured"
        );
        loop {
            delay.delay_millis(5_000);
        }
    }

    // Identity first: this is what tells SIM7600CE-T from a wrong module.
    send(&mut uart, &delay, "ATE0", 500);
    send(&mut uart, &delay, "ATI", 1_000);
    send(&mut uart, &delay, "AT+CGMR", 1_000);
    send(&mut uart, &delay, "AT+CSUB", 1_000);

    // Below this line everything is expected to fail without a card/antenna.
    send(&mut uart, &delay, "AT+CPIN?", 1_000);
    send(&mut uart, &delay, "AT+CSQ", 1_000);
    send(&mut uart, &delay, "AT+CREG?", 1_000);
    send(&mut uart, &delay, "AT+CPSI?", 1_000);
    send(&mut uart, &delay, "AT+CGPS?", 1_000);

    esp_println::println!(
        "\nlink is up. CSQ poll every 5 s (99,99 = no antenna/SIM, expected):"
    );
    loop {
        send(&mut uart, &delay, "AT+CSQ", 1_500);
        delay.delay_millis(3_500);
    }
}

/// Sends one command and echoes the module's reply line by line.
/// Returns true when a bare `OK` was seen within `wait_ms`.
fn send(uart: &mut Uart<'_, esp_hal::Blocking>, delay: &Delay, cmd: &str, wait_ms: u32) -> bool {
    let _ = uart.write(cmd.as_bytes());
    let _ = uart.write(b"\r\n");
    let _ = uart.flush();

    let mut line: String<128> = String::new();
    let mut ok = false;

    for _ in 0..(wait_ms / 10) {
        let mut buf = [0u8; 64];
        if let Ok(n) = uart.read_buffered(&mut buf) {
            for &b in &buf[..n] {
                match b {
                    b'\r' => {}
                    b'\n' => {
                        if !line.is_empty() {
                            esp_println::println!("  {}", line);
                            if line == "OK" {
                                ok = true;
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
        // A reply can arrive in pieces; keep reading past the OK.
        delay.delay_millis(10);
    }

    if !line.is_empty() {
        esp_println::println!("  {}", line);
    }
    ok
}
