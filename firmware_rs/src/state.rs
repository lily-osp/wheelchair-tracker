//! Persisted state that must survive deep sleep.
//!
//! Deep sleep resets the CPU and main RAM, so anything that must outlive a
//! sleep cycle lives in RTC fast memory (`#[esp_hal::ram(persistent)]` on the
//! static in `main.rs`) and is validated with a magic word plus a checksum:
//! a reset that happens while the state is being written must not boot with
//! half-written nonsense pretending to be valid.

use bytemuck::{Pod, Zeroable};

/// Monotonic milliseconds accumulated by the firmware itself.
///
/// The chip's RTC timer would also survive deep sleep, but this accumulator
/// works everywhere: each timer wake adds the duration it was programmed for,
/// so `mono_ms` moves at the same rate across wakes without depending on
/// which timer drivers this chip exposes.
pub type MonoMs = u64;

/// Magic word "WCTR".
pub const STATE_MAGIC: u32 = 0x5743_5452;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Pod, Zeroable)]
#[repr(C)]
pub struct PersistState {
    pub magic: u32,
    /// CRC16-CCITT over the struct bytes with this field zeroed.
    pub checksum: u16,
    _pad: u16,

    /// Degrees x 1e7; no floats in RTC memory.
    pub lat_e7: i32,
    pub lon_e7: i32,
    /// Wall clock (unix, s) of the fix, from the GNSS sentence when available.
    pub unix_fix: u32,

    /// Number of satellites the fix was acquired with, 0 if unknown.
    pub sats: u8,
    /// 0/1 flags - `bool` is not `AnyBitPattern` in bytemuck, and RTC
    /// memory must be plain bytes.
    pub moving: u8,
    /// Set after the first full classification, so a chair that boots parked
    /// does not buy a GNSS fix on the way to deciding it is parked.
    pub settled: u8,
    pub had_fix: u8,

    /// Monotonic ms when the fix was acquired.
    pub fix_mono_ms: MonoMs,
    /// Monotonic ms of the next due report.
    pub next_report_ms: MonoMs,
    /// Accumulated monotonic ms (awake + slept), advanced by the caller.
    pub mono_ms: MonoMs,
    /// Absolute mono_ms the sleep deadline was armed for; a timer wake
    /// restores `mono_ms` to exactly this.
    pub sleep_target_ms: MonoMs,

    /// Motion hysteresis streaks; in RTC memory, because the CPU resets on
    /// every wake and function-local statics are lost (this is issue #2).
    pub moving_streak: u8,
    pub idle_streak: u8,
    _pad3: [u8; 6],
}

impl PersistState {
    pub const ZEROED: Self = Self {
        magic: 0,
        checksum: 0,
        _pad: 0,
        lat_e7: 0,
        lon_e7: 0,
        unix_fix: 0,
        sats: 0,
        moving: 0,
        settled: 0,
        had_fix: 0,
        fix_mono_ms: 0,
        next_report_ms: 0,
        mono_ms: 0,
        sleep_target_ms: 0,
        moving_streak: 0,
        idle_streak: 0,
        _pad3: [0; 6],
    };

    /// Returns true when the magic and checksum are valid.
    pub fn is_valid(&self) -> bool {
        if self.magic != STATE_MAGIC {
            return false;
        }
        let mut copy = *self;
        copy.checksum = 0;
        crc16_ccitt(bytemuck::bytes_of(&copy)) == self.checksum
    }

    /// Recomputes the checksum after mutating the state.
    pub fn seal(&mut self) {
        self.checksum = 0;
        self.checksum = crc16_ccitt(bytemuck::bytes_of(self));
    }

    /// Wipes the state to first-boot condition (no fix, parked, unset).
    /// Callers set `next_report_ms` explicitly afterwards.
    pub fn reset(&mut self) {
        *self = Self::ZEROED;
        self.magic = STATE_MAGIC;
        self.seal();
    }

    pub fn fix_age_ms(&self, now_ms: MonoMs) -> MonoMs {
        now_ms.saturating_sub(self.fix_mono_ms)
    }

    pub fn is_moving(&self) -> bool {
        self.moving != 0
    }
    pub fn set_moving(&mut self, v: bool) {
        self.moving = v as u8;
    }
    pub fn is_settled(&self) -> bool {
        self.settled != 0
    }
    pub fn set_settled(&mut self, v: bool) {
        self.settled = v as u8;
    }
    pub fn has_fix(&self) -> bool {
        self.had_fix != 0
    }
    pub fn set_had_fix(&mut self, v: bool) {
        self.had_fix = v as u8;
    }
}

/// CRC16-CCITT (poly 0x1021, init 0xFFFF).
pub fn crc16_ccitt(bytes: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in bytes {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reset_produces_valid_state() {
        let mut s = PersistState::ZEROED;
        s.reset();
        assert!(s.is_valid());
        assert_eq!(s.magic, STATE_MAGIC);
        assert!(!s.has_fix() && !s.is_moving() && !s.is_settled());
    }

    #[test]
    fn mutation_without_seal_breaks_validation() {
        let mut s = PersistState::ZEROED;
        s.reset();
        s.set_had_fix(true);
        assert!(!s.is_valid(), "torn write must not validate");
        s.seal();
        assert!(s.is_valid());
    }

    #[test]
    fn garbage_magic_rejected() {
        let s = PersistState::ZEROED; // magic 0
        assert!(!s.is_valid());
    }

    #[test]
    fn crc_known_vector() {
        // CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF): empty -> init value;
        // the single zero byte hashes to 0xE1F0.
        assert_eq!(crc16_ccitt(&[]), 0xFFFF);
        assert_eq!(crc16_ccitt(&[0]), 0xE1F0);
    }

    #[test]
    fn fix_age_saturates() {
        let mut s = PersistState::ZEROED;
        s.reset();
        s.fix_mono_ms = 100;
        s.seal();
        assert_eq!(s.fix_age_ms(900), 800);
        assert_eq!(s.fix_age_ms(50), 0);
    }
}

// Safety: `PersistState` is `Pod` (padding-free, valid for any bit pattern),
// which satisfies `Persistable`'s contract for RTC-persistent statics.
#[cfg(feature = "firmware")]
unsafe impl esp_hal::Persistable for PersistState {}
