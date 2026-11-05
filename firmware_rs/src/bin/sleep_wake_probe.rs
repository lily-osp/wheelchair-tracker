//! Bench probe: deep-sleep wake from LIS2DH INT1 (GPIO0).
//!
//! - Power-on/reset: configures LIS2DH @0x18, arms INT1 high-level wake
//!   (GPIO0, pull-down, low-power path), then enters deep sleep (10 min
//!   timer watchdog).
//! - On GPIO wake (shake/tilt): boots, detects wake cause, prints success,
//!   and stays awake so USB-JTAG serial can be captured.
//!
//!   cd firmware_rs && cargo build --bin sleep_wake_probe
//!   espflash flash --chip esp32c3 target/.../debug/sleep_wake_probe
//!   python3 /var/tmp/scratch/serial_read.py /dev/ttyACM0 300
//!   (then shake/tilt the board to trigger INT1)

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Event, Input, InputConfig, Pull, WakeupConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::rtc_cntl::sleep::{LowPower, RtcSleepConfig};
use esp_hal::rtc_cntl::{WakeupSource, wakeup_cause};
use esp_hal::timer::timg::TimerGroup;

use firmware_rs::config as cfg;
use firmware_rs::lis2dh::Lis2dh;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    let cause = wakeup_cause();
    esp_println::println!("sleep_wake_probe: wake cause={:?}", cause);

    if cause.contains(WakeupSource::Gpio) {
        esp_println::println!("WAKE=GPIO (LIS2DH INT1 deep-sleep interception OK)");
        esp_println::println!("staying awake; shake again won't re-sleep in this mode");
        // Keep USB-JTAG observable.
        let delay = Delay::new();
        loop {
            esp_println::println!("awake (GPIO wake confirmed)");
            delay.delay_millis(5_000);
        }
    }

    // Power-on / timer / other: configure IMU and sleep.
    let mut imu_int = Input::new(
        peripherals.GPIO0,
        InputConfig::default().with_pull(Pull::Down),
    );
    let mut i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(peripherals.GPIO8)
        .with_scl(peripherals.GPIO9);

    // Use bench-verified address 0x18 (SA0 low); fall back to 0x19.
    let mut addr = 0x18u8;
    {
        let mut imu = Lis2dh::new(&mut i2c, addr);
        if !matches!(imu.probe(), Ok(true)) {
            addr = 0x19;
            let mut imu = Lis2dh::new(&mut i2c, addr);
            if !matches!(imu.probe(), Ok(true)) {
                esp_println::println!("imu not found at 0x18/0x19; cannot arm wake");
                let delay = Delay::new();
                loop {
                    delay.delay_millis(5_000);
                }
            }
        }
        let mut imu = Lis2dh::new(&mut i2c, addr);
        let _ = imu.configure(cfg::IMU_THRESHOLD, cfg::IMU_DURATION);
        let _ = imu.clear_interrupt();
        esp_println::println!(
            "imu configured @0x{:02x} threshold={} duration={}",
            addr, cfg::IMU_THRESHOLD, cfg::IMU_DURATION
        );
    }

    // Arm INT1 wake: active-high, pull-down, low-power path.
    imu_int.listen(Event::HighLevel);
    let _ = imu_int.apply_wakeup_config(&WakeupConfig::default().with_low_power_path(true));
    let _ = Lis2dh::new(&mut i2c, addr).clear_interrupt();

    let sleep_ms: u64 = 10 * 60 * 1000; // 10 min watchdog
    esp_println::println!("entering deep sleep for {} ms; SHAKE/TILT to wake via LIS2 INT1", sleep_ms);

    let mut lp = LowPower::new(peripherals.LPWR);
    lp.set_wakeup_deadline(
        esp_hal::time::Instant::now() + esp_hal::time::Duration::from_millis(sleep_ms),
    );
    lp.sleep_deep(RtcSleepConfig::default())
}
