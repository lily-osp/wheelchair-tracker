//! Bench probe: uplink mode switch on `cfg::MODE_SWITCH` (GPIO10).
//!
//! Prints the sampled level and the channel it selects, once a second, so the
//! wiring and the switch polarity can be checked without the modem, the IMU
//! or deep sleep in the way.
//!
//!   cd firmware_rs && cargo build --bin mode_probe
//!   espflash flash --chip esp32c3 target/.../debug/mode_probe
//!   python3 /var/tmp/scratch/serial_read.py /dev/ttyACM0 60
//!
//! Expected: switch open -> `level=high via=http`, closed to GND ->
//! `level=low via=sms`. A level that follows nothing is an unwired pin; a
//! level stuck low with the switch open is a short to ground.

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, InputConfig, Level, Pull};

use firmware_rs::config as cfg;
use firmware_rs::config::ReportVia;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();

    let switch = Input::new(p.GPIO10, InputConfig::default().with_pull(Pull::Up));
    esp_println::println!(
        "mode_probe: switch=GPIO{} fitted={} pull=up (closed=GND=sms)",
        cfg::MODE_SWITCH,
        cfg::MODE_SWITCH_FITTED
    );

    let mut last: Option<ReportVia> = None;
    loop {
        // Same debounce as the firmware: three samples 2 ms apart.
        let mut low = 0u8;
        for i in 0..3 {
            if i > 0 {
                delay.delay_millis(2);
            }
            low += (switch.level() == Level::Low) as u8;
        }
        let via = match low {
            0 => Some(ReportVia::Http),
            3 => Some(ReportVia::Sms),
            _ => None,
        };

        match via {
            Some(v) => {
                esp_println::println!(
                    "level={} via={}{}",
                    if v == ReportVia::Sms { "low" } else { "high" },
                    v.name(),
                    if last.is_some_and(|l| l != v) {
                        " CHANGED"
                    } else {
                        ""
                    }
                );
                last = Some(v);
            }
            None => esp_println::println!("level=bounce via=unchanged"),
        }
        delay.delay_millis(1000);
    }
}
