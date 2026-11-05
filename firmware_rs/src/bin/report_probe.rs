//! Bench probe: full report path (modem + GNSS fix + AT+HTTP POST) without
//! deep sleep, so USB-JTAG serial stays observable.
//!
//! Mirrors `main.rs`'s report payload (`value` + `lat`/`lon`/`created_at`) and
//! uses the same production `Sim7600` + `uplink::post_json` path.
//!
//!   cd firmware_rs && cargo build --bin report_probe
//!   espflash flash --chip esp32c3 target/.../debug/report_probe
//!   python3 /var/tmp/scratch/serial_read.py /dev/ttyACM0 240

#![no_std]
#![no_main]

use core::fmt::Write as _;

use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use heapless::String;
use static_cell::StaticCell;

use firmware_rs::config as cfg;
use firmware_rs::gpsinfo::GpsFix;
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

    esp_println::println!("report_probe: waiting for AT...");
    if !sim.sync(40, 500).await {
        esp_println::println!("AT: no answer; run modem_probe first");
        idle().await;
    }
    let up = sim.bring_up(Some(cfg::APN)).await;
    esp_println::println!("bring_up={} apn=\"{}\" sim={}", up, cfg::APN, sim.sim_status().await);
    esp_println::println!(
        "csq={} dBm registered={} attached={}",
        sim.signal().await,
        sim.is_registered().await,
        sim.is_attached().await
    );

    esp_println::println!("acquiring GNSS fix (deadline {} ms)...", cfg::FIX_DEADLINE_MS);
    let fix = acquire_fix(&mut sim).await;
    match fix {
        Some(f) => esp_println::println!(
            "fix=OK lat={} lon={} sats={} unix={}",
            deg_str(f.lat_e7),
            deg_str(f.lon_e7),
            f.sats,
            f.unix_time
        ),
        None => esp_println::println!("fix=NONE (posting with last-known/zero as probe marker)"),
    }

    let (lat_e7, lon_e7, sats, unix_fix) = fix
        .map(|f| (f.lat_e7, f.lon_e7, f.sats, f.unix_time))
        .unwrap_or((0, 0, 0, 0));

    let mut body = String::<160>::new();
    let _ = write!(
        &mut body,
        "M {},{} s{} a0s report-probe",
        deg_str(lat_e7),
        deg_str(lon_e7),
        sats
    );

    let mut loc_body = String::<256>::new();
    let _ = write!(
        &mut loc_body,
        "{{\"value\":\"{}\",\"lat\":{},\"lon\":{},\"created_at\":{}}}",
        body.as_str(),
        deg_str(lat_e7),
        deg_str(lon_e7),
        unix_fix
    );

    let mut url = String::<320>::new();
    let _ = write!(
        &mut url,
        "http://{}/api/v2/{}/feeds/{}/data?X-AIO-Key={}",
        cfg::SERVER_HOST,
        cfg::AIO_USER,
        cfg::AIO_FEED_LOCATION,
        cfg::AIO_KEY
    );

    esp_println::println!("posting payload: {}", loc_body.as_str());
    match uplink::post_json(&mut sim, url.as_str(), loc_body.as_bytes()).await {
        Ok(status) => esp_println::println!("http={}", status),
        Err(e) => esp_println::println!("http={:?}", e),
    }

    let _ = sim.gnss_off(4000).await;
    let _ = sim.ok("AT+CPOF").await;
    idle().await
}

async fn acquire_fix(sim: &mut Sim7600<'_>) -> Option<GpsFix> {
    if !sim.gnss_on().await {
        esp_println::println!("gnss_on failed");
        return None;
    }
    let deadline = Instant::now() + Duration::from_millis(cfg::FIX_DEADLINE_MS as u64);
    while Instant::now() < deadline {
        if let Some(fix) = sim.position().await {
            if fix.valid() {
                return Some(fix);
            }
            esp_println::println!("fix=not-yet lat_e7={} lon_e7={}", fix.lat_e7, fix.lon_e7);
        }
        Timer::after_millis(2000).await;
    }
    None
}

/// Degrees x 1e7 -> "-106.8452300" style fixed-point string. No floats.
fn deg_str(e7: i32) -> String<16> {
    let mut out = String::new();
    if e7 < 0 {
        let _ = out.push('-');
    }
    let a = e7.unsigned_abs();
    let _ = write!(&mut out, "{}.", a / 10_000_000);
    let _ = write!(&mut out, "{:07}", a % 10_000_000);
    out
}

async fn idle() -> ! {
    loop {
        Timer::after_millis(60_000).await;
    }
}
