//! Wheelchair location tracker firmware.
//!
//! ESP32-C3 SuperMini + SIM7600CE-T + LIS2DH, esp-hal based (`no_std`).
//! Fixes for tracker issues #1-#6 are design requirements here, not patches:
//!
//! - #1: state lives in RTC-fast memory via `#[esp_hal::ram(persistent)]`
//!   (`bin/main.rs`), validated with magic + CRC16 (`state.rs`).
//! - #2: motion hysteresis streaks are part of the persisted state
//!   (`motion.rs`), never function-local statics.
//! - #3: timestamps are a firmware-accumulated monotonic counter stored in
//!   the persisted state (`state.rs::mono_ms`), not `millis()`.
//! - #4: modem power-on is idempotent (`AT` probe before any PWRKEY pulse).
//! - #5: PWRKEY gate idle LOW, pulsed HIGH 1500 ms (N-MOSFET, per
//!   hardware.md); the level is set at `Output::new()` construction time.
//! - #6: GNSS via `AT+CGNSSINFO`/`AT+CGPSINFO` on the single AT port; no
//!   second UART, no NMEA parser.
//!
//! Uplink: SMS by default (verified path). `Http` runs PPP over the modem's
//! AT UART (`ATD*99***1#`) with embassy-net - the ESP32 radio is unused.

#![cfg_attr(not(test), no_std)]

pub mod at;
pub mod config;
pub mod gpsinfo;
pub mod lis2dh;
pub mod motion;
pub mod state;

#[cfg(feature = "firmware")]
pub mod sim7600;
#[cfg(feature = "firmware")]
pub mod uplink;
