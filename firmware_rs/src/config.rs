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
/// Uplink mode switch: SPST to GND on GPIO10. Closed (pin LOW) = SMS,
/// open or not fitted (internal pull-up, pin HIGH) = internet/HTTP, so a
/// board without the switch keeps the HTTP default.
///
/// The internal pull-up (~45 k) would burn ~73 uA through a closed switch,
/// more than the whole deep-sleep budget, so the firmware samples the pin
/// with the pull-up enabled and then drops it to `Pull::None` before sleep.
/// An external 1 M pull-up (~3 uA) is the alternative if the pad's leakage
/// ever proves to float on your board.
pub const MODE_SWITCH: u8 = 10;
/// Set false to ignore the pin entirely and use `REPORT_VIA_DEFAULT`.
pub const MODE_SWITCH_FITTED: bool = true;

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
/// Report while travelling, HTTP mode.
pub const SEND_MOVING_MS: u32 = 60_000;
/// Parked heartbeat (15 min), HTTP mode.
pub const SEND_IDLE_MS: u32 = 900_000;
/// Report while travelling, SMS mode. Slower than HTTP on purpose: every
/// message is billed, so 5 min is 12/hour instead of 60/hour. Recalculate
/// against the actual per-SMS tariff before shipping.
pub const SEND_MOVING_SMS_MS: u32 = 300_000;
/// Parked heartbeat (1 h), SMS mode.
pub const SEND_IDLE_SMS_MS: u32 = 3_600_000;
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

/// Uplink channel, selected at every wake by the `MODE_SWITCH` pin.
///
/// - `Http` posts through the module's `AT+HTTP` stack; verified on hardware
///   2026-11-05 against Adafruit IO (HTTP 200). Cheap per report and carries
///   the full JSON point, but needs a data-capable SIM with credit and
///   coverage good enough to attach a PDP context.
/// - `Sms` needs only voice-grade coverage and no APN, which is why it is the
///   fallback channel, but it costs money per message and delivers one short
///   text instead of a structured point. It needs `owner` in adafruit.toml.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ReportVia {
    /// Discriminant 0, so zeroed RTC state reads as the default channel.
    Http = 0,
    Sms = 1,
}

impl ReportVia {
    /// Switch closed (pin pulled to GND) selects SMS.
    pub const fn from_switch(pin_low: bool) -> Self {
        if pin_low { Self::Sms } else { Self::Http }
    }

    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    pub const fn from_u8(v: u8) -> Self {
        if v == Self::Sms as u8 {
            Self::Sms
        } else {
            Self::Http
        }
    }

    pub const fn name(self) -> &'static str {
        match self {
            Self::Http => "http",
            Self::Sms => "sms",
        }
    }

    /// Report cadence while travelling.
    pub const fn moving_ms(self) -> u32 {
        match self {
            Self::Http => SEND_MOVING_MS,
            Self::Sms => SEND_MOVING_SMS_MS,
        }
    }

    /// Parked heartbeat cadence.
    pub const fn idle_ms(self) -> u32 {
        match self {
            Self::Http => SEND_IDLE_MS,
            Self::Sms => SEND_IDLE_SMS_MS,
        }
    }
}

/// Used when `MODE_SWITCH_FITTED` is false, or when the sampled level is
/// inconsistent and no previous mode is on record.
pub const REPORT_VIA_DEFAULT: ReportVia = ReportVia::Http;

/// How long to wait for network registration before giving up.
pub const REGISTRATION_TIMEOUT_MS: u32 = 60_000;
/// Deadline for a GNSS acquisition attempt (cold start ~35 s).
pub const FIX_DEADLINE_MS: u32 = 45_000;
/// Deadline for one HTTP POST, from `AT+HTTPINIT` to `+HTTPACTION`.
pub const UPLINK_DEADLINE_MS: u32 = 60_000;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn switch_closed_selects_sms() {
        assert_eq!(ReportVia::from_switch(true), ReportVia::Sms);
        assert_eq!(ReportVia::from_switch(false), ReportVia::Http);
    }

    #[test]
    fn zeroed_state_byte_reads_as_default_channel() {
        assert_eq!(ReportVia::from_u8(0), REPORT_VIA_DEFAULT);
        assert_eq!(ReportVia::from_u8(1), ReportVia::Sms);
        // Garbage from a torn/older state must not pick the billed channel.
        assert_eq!(ReportVia::from_u8(0xFF), ReportVia::Http);
    }

    #[test]
    fn round_trips_through_the_persisted_byte() {
        for via in [ReportVia::Http, ReportVia::Sms] {
            assert_eq!(ReportVia::from_u8(via.as_u8()), via);
        }
    }

    #[test]
    fn sms_cadence_is_never_faster_than_http() {
        assert!(ReportVia::Sms.moving_ms() >= ReportVia::Http.moving_ms());
        assert!(ReportVia::Sms.idle_ms() >= ReportVia::Http.idle_ms());
        // Every sleep is clamped to the watchdog, so a cadence longer than
        // the backstop would silently become the backstop.
        assert!(ReportVia::Sms.idle_ms() <= WATCHDOG_MS);
    }
}
