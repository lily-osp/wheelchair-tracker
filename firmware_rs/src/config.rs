//! Board and cadence configuration.
//!
//! Verify every pin against the board's silkscreen before flashing.
//! GPIO18/19 are USB Serial/JTAG on the C3 SuperMini and are unusable when
//! programming over USB - the only path on this board.

/// C3 TX -> modem RXD.
pub const MODEM_TX: u8 = 4;
/// C3 RX <- modem TXD.
pub const MODEM_RX: u8 = 5;
/// N-MOSFET gate driving the modem PWRKEY, never PWRKEY direct.
pub const PWRKEY_GATE: u8 = 6;
/// I2C SDA. The onboard LED on GPIO8 must be removed (see root README).
pub const IMU_SDA: u8 = 8;
/// I2C SCL.
pub const IMU_SCL: u8 = 9;
/// LIS2DH INT1, RTC pin 0-5 wakes deep sleep on the C3.
pub const IMU_INT1: u8 = 0;
/// 0x19 with SDO/SA0 high, 0x18 pulled low. Bench-verified at 0x18
/// (SA0/SDO low) on 2026-11-05.
pub const IMU_ADDR: u8 = 0x18;
/// 250 mg at 16 mg/digit in LIS2DH low-power mode.
pub const IMU_THRESHOLD: u8 = 16;
pub const IMU_DURATION: u8 = 2;

/// PWRKEY pulse length.
pub const MODEM_PWRKEY_HOLD_MS: u32 = 1500;
/// Sync is attempts x (retries + 1) x 1 s: `command()` retries a timeout
/// internally. Kept short so a missing modem is reported quickly.
pub const MODEM_SYNC_ATTEMPTS: u8 = 8;

/// Motion sampling interval.
pub const POLL_MS: u32 = 5000;
/// Report while travelling.
pub const SEND_MOVING_MS: u32 = 60_000;
/// Parked heartbeat (15 min).
pub const SEND_IDLE_MS: u32 = 900_000;
/// Always-on backstop (1 h): a tracker that can only wake on motion is a
/// tracker that goes silent the day the sensor fails.
pub const WATCHDOG_MS: u32 = 3_600_000;
/// Fix reusable for 5 min.
pub const FIX_VALID_MS: u32 = 300_000;
/// Beyond this, label the report stale.
pub const FIX_STALE_MS: u32 = 600_000;

/// Hysteresis enter threshold, mg of deviation from 1 g. Without this a chair
/// stopped at a kerb flips state every few seconds.
pub const MOVE_ENTER_MG: i32 = 250;
/// Hysteresis exit band: |magnitude - 1000| must be within this.
pub const MOVE_EXIT_MG: i32 = 120;
/// Consecutive motion samples (or wakes) before a state transition.
pub const HYSTERESIS_SAMPLES: u8 = 3;

// Adafruit IO settings come from adafruit.toml via build.rs (aio_config.rs).
include!(concat!(env!("OUT_DIR"), "/aio_config.rs"));
pub const OWNER: &str = CFG_OWNER;
/// Carrier APN for the PDP context; empty means "leave the SIM's default".
pub const APN: &str = CFG_APN;

/// Adafruit IO REST endpoint (plain HTTP; no TLS in this firmware).
pub const SERVER_HOST: &str = CFG_AIO_HOST;
pub const SERVER_PORT: u16 = CFG_AIO_PORT;
/// Adafruit IO key for the X-AIO-Key header.
pub const AIO_KEY: &str = CFG_AIO_KEY;
/// Adafruit IO username, used in feed paths.
pub const AIO_USER: &str = CFG_AIO_USER;
/// Adafruit IO feed names.
pub const AIO_FEED_LOCATION: &str = CFG_AIO_FEED_LOCATION;
pub const AIO_FEED_TIME: &str = CFG_AIO_FEED_TIME;

/// Uplink channel.
///
/// - `Http` posts through the module's `AT+HTTP` stack; verified on hardware
///   2026-11-05 against Adafruit IO (HTTP 200). Plain HTTP: the key is sent
///   in the clear until the TLS path is enabled.
/// - `Sms` is the fallback and needs `owner` in adafruit.toml.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReportVia {
    Sms,
    Http,
}

pub const REPORT_VIA: ReportVia = ReportVia::Http;

/// How long to wait for network registration before giving up.
pub const REGISTRATION_TIMEOUT_MS: u32 = 60_000;
/// Deadline for a GNSS acquisition attempt (cold start ~35 s).
pub const FIX_DEADLINE_MS: u32 = 45_000;
/// Deadline for one HTTP POST, from `AT+HTTPINIT` to `+HTTPACTION`.
pub const UPLINK_DEADLINE_MS: u32 = 60_000;
