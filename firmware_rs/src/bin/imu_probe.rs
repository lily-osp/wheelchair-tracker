//! Bench probe: LIS2DH I2C + INT1.
//!
//! Verifies the sensor is wired correctly (WHO_AM_I/probe), reads magnitude,
//! and shows INT1 pin/latch state. No modem, no sleep.
//!
//!   cd firmware_rs && cargo build --bin imu_probe
//!   espflash flash --chip esp32c3 target/.../debug/imu_probe
//!   python3 /var/tmp/scratch/serial_read.py /dev/ttyACM0 60
//!
//! Tilt/shake the board to see magnitude move away from ~1000 mg and INT1
//! activity (after configure arms the threshold interrupt).

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, InputConfig, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};

use firmware_rs::config as cfg;
use firmware_rs::lis2dh::Lis2dh;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();

    let imu_int = Input::new(p.GPIO0, InputConfig::default().with_pull(Pull::Down));
    let mut i2c = I2c::new(p.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(p.GPIO8)
        .with_scl(p.GPIO9);

    esp_println::println!(
        "imu_probe: SDA=GPIO{} SCL=GPIO{} INT1=GPIO0 (trying 0x19, then 0x18)",
        cfg::IMU_SDA,
        cfg::IMU_SCL
    );

    let mut found_addr = None;
    for &addr in &[0x19u8, 0x18u8] {
        // Read WHO_AM_I raw for diagnostics.
        let mut who = [0u8; 1];
        let who_res = {
            use embedded_hal::i2c::I2c;
            i2c.write_read(addr, &[0x0F], &mut who)
        };
        match who_res {
            Ok(()) => esp_println::println!("addr 0x{:02x}: WHO_AM_I=0x{:02x}", addr, who[0]),
            Err(_) => esp_println::println!("addr 0x{:02x}: WHO_AM_I read NACK/bus error", addr),
        }
        let res = {
            let mut imu = Lis2dh::new(&mut i2c, addr);
            imu.probe()
        };
        if matches!(res, Ok(true)) {
            esp_println::println!("probe=OK at 0x{:02x}", addr);
            found_addr = Some(addr);
            break;
        }
    }
    let Some(addr) = found_addr else {
        esp_println::println!("no LIS2DH found at 0x19/0x18; check VCC/GND/SDA/SCL/pullups, SA0, and GPIO8 onboard LED removal");
        idle(&delay);
    };

    {
        let mut imu = Lis2dh::new(&mut i2c, addr);
        if imu.configure(cfg::IMU_THRESHOLD, cfg::IMU_DURATION).is_err() {
            esp_println::println!("configure=BUS-ERROR");
            idle(&delay);
        }
        esp_println::println!(
            "configured @0x{:02x}: threshold={} (1/16 g), duration={} samples",
            addr, cfg::IMU_THRESHOLD, cfg::IMU_DURATION
        );
        let _ = imu.clear_interrupt();
    }

    // Dump key regs for diagnosis.
    for (name, reg) in [
        ("CTRL1", 0x20u8), ("CTRL4", 0x23u8), ("STATUS", 0x27u8),
        ("INT1_CFG", 0x30u8), ("INT1_SRC", 0x31u8), ("CTRL3", 0x22u8),
    ] {
        use embedded_hal::i2c::I2c;
        let mut v = [0u8; 1];
        let ok = i2c.write_read(addr, &[reg], &mut v).is_ok();
        if ok { esp_println::println!("  {}(0x{:02x})=0x{:02x}", name, reg, v[0]); }
        else { esp_println::println!("  {} read err", name); }
    }

    esp_println::println!("streaming mag + GPIO0(INT1) level; SHAKE/TILT now (INT1_SRC read/cleared only on change)");
    let mut last_src = 0u8;
    loop {
        let mag = {
            let mut imu = Lis2dh::new(&mut i2c, addr);
            imu.magnitude(200)
        };
        let int1_level = imu_int.is_high();
        // Read SRC to observe latch, but note: reading clears it (pin may drop).
        // Sample GPIO0 first; also show SRC.
        use embedded_hal::i2c::I2c;
        let mut src = [0u8; 1];
        let _ = i2c.write_read(addr, &[0x31], &mut src);
        if int1_level || src[0] != last_src || (mag - 1000).abs() > 150 {
            esp_println::println!("mag={} delta={} int1_gpio0={} src=0x{:02x}", mag, (mag - 1000).abs(), int1_level, src[0]);
        }
        last_src = src[0];
        delay.delay_millis(100);
    }
}

fn idle(delay: &Delay) -> ! {
    loop {
        delay.delay_millis(5_000);
    }
}
