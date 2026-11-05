//! Bench probe #2: the real async driver + the real HTTP uplink.
//!
//! Unlike `http_probe` (raw blocking UART), this one goes through
//! `Sim7600` and `uplink::post_json`, so a `status=200` here means the
//! production report path works. Run after `modem_probe` passes.
//!
//!   cargo build --bin net_probe && espflash flash ... net_probe
//!
//! Needs `adafruit.toml` with a real key/user and `apn` set for the SIM.
//! Posts one point `{"value":"net-probe"}` to the location feed.

#![no_std]
#![no_main]

use core::fmt::Write as _;

use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use heapless::String;
use static_cell::StaticCell;

use firmware_rs::config as cfg;
use firmware_rs::sim7600::Sim7600;
use firmware_rs::uplink;

esp_bootloader_esp_idf::esp_app_desc!();

static MODEM_UART: StaticCell<Uart<'static, esp_hal::Async>> = StaticCell::new();

#[esp_rtos::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    // Auto power-on via PWR-3V3; held low and never pulsed.
    let _pwrkey = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());

    let uart = MODEM_UART.init(
        Uart::new(
            peripherals.UART1,
            UartConfig::default().with_baudrate(115_200),
        )
        .unwrap()
        .with_rx(peripherals.GPIO5)
        .with_tx(peripherals.GPIO4)
        .into_async(),
    );
    let mut sim = Sim7600::new(uart, Duration::from_millis(1500), 2);

    esp_println::println!("net_probe: waiting for AT...");
    if !sim.sync(40, 500).await {
        esp_println::println!("AT: no answer; run modem_probe first");
        idle().await;
    }

    let up = sim.bring_up(Some(cfg::APN)).await;
    esp_println::println!("bring_up={} apn=\"{}\"", up, cfg::APN);
    esp_println::println!("sim={}", sim.sim_status().await);
    esp_println::println!("csq={} dBm", sim.signal().await);

    let mut registered = sim.is_registered().await;
    for _ in 0..30 {
        if registered {
            break;
        }
        Timer::after_millis(2_000).await;
        registered = sim.is_registered().await;
    }
    esp_println::println!(
        "registered={} attached={}",
        registered,
        sim.is_attached().await
    );

    // Surface the PDP address: no address here means the carrier never gave
    // the module an IPv4 context, and every HTTP call would fail.
    let addr = sim.command("AT+CGPADDR=1", None).await;
    for line in addr.lines.iter() {
        esp_println::println!("  {}", line);
    }

    let mut url = String::<320>::new();
    let _ = write!(
        &mut url,
        "http://{}/api/v2/{}/feeds/{}/data?X-AIO-Key={}",
        cfg::SERVER_HOST,
        cfg::AIO_USER,
        cfg::AIO_FEED_LOCATION,
        cfg::AIO_KEY
    );

    esp_println::println!("posting to {}/api/v2/{}/feeds/{}/data...", cfg::SERVER_HOST, cfg::AIO_USER, cfg::AIO_FEED_LOCATION);
    let body = br#"{"value":"net-probe"}"#;
    match uplink::post_json(&mut sim, url.as_str(), body).await {
        Ok(status) => esp_println::println!("POST {}: Adafruit IO accepted the point", status),
        Err(e) => esp_println::println!("POST failed: {:?}", e),
    }

    idle().await
}

async fn idle() -> ! {
    loop {
        Timer::after_millis(60_000).await;
    }
}
