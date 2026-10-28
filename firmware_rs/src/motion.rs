//! Motion gate logic.
//!
//! Pure decision logic, split from I/O so it is host-testable. The
//! accelerometer answers one question: is the chair moving? The hysteresis
//! streaks live in the persisted state, because the CPU resets on every wake
//! (issue #2 in the tracker repo) and function-local statics never survive.

use crate::state::{MonoMs, PersistState};

/// Result of a hysteresis update.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionEvent {
    None,
    BecameMoving,
    BecameParked,
}

/// Whether the chair counts as active for this wake.
///
/// Combine both sources: the latched INT1 threshold interrupt (fast, but a
/// lost interrupt must not silently read as parked) and the polled
/// multi-sample magnitude check.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WakeMotion {
    pub int1_latched: bool,
    pub polled_moving: bool,
}

impl WakeMotion {
    pub fn active(&self) -> bool {
        self.int1_latched || self.polled_moving
    }
}

/// Hysteresis parameters (from config, kept here for testability).
#[derive(Debug, Clone, Copy)]
pub struct MotionParams {
    pub enter_mg: i32,
    pub exit_mg: i32,
    pub samples: u8,
}

/// Updates the motion state with this wake's sample.
///
/// - Entering MOVING needs `samples` consecutive active wakes.
/// - Leaving needs `samples` consecutive idle wakes where the magnitude sat
///   inside the exit band, so a chair that got bumped once does not flip back
///   while the bump is still ringing.
pub fn update(
    state: &mut PersistState,
    motion: WakeMotion,
    magnitude_mg: i32,
    params: &MotionParams,
) -> MotionEvent {
    let active = motion.active();
    if active {
        state.moving_streak = state.moving_streak.saturating_add(1);
        state.idle_streak = 0;
    } else {
        state.idle_streak = state.idle_streak.saturating_add(1);
        state.moving_streak = 0;
    }

    if !state.is_moving() && state.moving_streak >= params.samples {
        state.set_moving(true);
        return MotionEvent::BecameMoving;
    }

    if state.is_moving()
        && state.idle_streak >= params.samples
        && (magnitude_mg - 1000).abs() <= params.exit_mg
    {
        state.set_moving(false);
        return MotionEvent::BecameParked;
    }

    MotionEvent::None
}

/// Computes the report cadence after an event or a completed report.
pub fn next_report_ms(moving: bool, now: MonoMs, moving_ms: MonoMs, idle_ms: MonoMs) -> MonoMs {
    now + if moving { moving_ms } else { idle_ms }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::STATE_MAGIC;

    fn state() -> PersistState {
        let mut s = PersistState::ZEROED;
        s.reset();
        s
    }

    const P: MotionParams = MotionParams {
        enter_mg: 250,
        exit_mg: 120,
        samples: 3,
    };

    fn active() -> WakeMotion {
        WakeMotion {
            int1_latched: true,
            polled_moving: false,
        }
    }

    fn idle(magnitude: i32) -> (WakeMotion, i32) {
        (
            WakeMotion {
                int1_latched: false,
                polled_moving: false,
            },
            magnitude,
        )
    }

    #[test]
    fn entering_moving_needs_streak() {
        let mut s = state();
        assert_eq!(update(&mut s, active(), 1000, &P), MotionEvent::None);
        assert_eq!(update(&mut s, active(), 1000, &P), MotionEvent::None);
        assert_eq!(
            update(&mut s, active(), 1000, &P),
            MotionEvent::BecameMoving
        );
        assert!(s.is_moving());
    }

    #[test]
    fn single_bump_does_not_flip_state() {
        let mut s = state();
        update(&mut s, active(), 1000, &P);
        let (m, mag) = idle(1000);
        assert_eq!(update(&mut s, m, mag, &P), MotionEvent::None);
        assert!(!s.is_moving(), "one bump must not enter MOVING");
    }

    #[test]
    fn active_streak_reset_by_idle() {
        let mut s = state();
        update(&mut s, active(), 1000, &P);
        update(&mut s, active(), 1000, &P);
        let (m, mag) = idle(1000);
        update(&mut s, m, mag, &P);
        update(&mut s, m, mag, &P);
        // Streak was reset; the next two actives must not be enough.
        assert_eq!(update(&mut s, active(), 1000, &P), MotionEvent::None);
        assert_eq!(update(&mut s, active(), 1000, &P), MotionEvent::None);
        assert_eq!(
            update(&mut s, active(), 1000, &P),
            MotionEvent::BecameMoving
        );
    }

    #[test]
    fn parking_needs_streak_and_stillness() {
        let mut s = state();
        for _ in 0..3 {
            update(&mut s, active(), 1300, &P);
        }
        assert!(s.is_moving());
        // Idle wakes but magnitude outside the exit band: still moving.
        let (m, mag) = idle(1300);
        assert_eq!(update(&mut s, m, mag, &P), MotionEvent::None);
        // Inside the band but streak not satisfied: still moving.
        let (m, mag) = idle(1000);
        assert_eq!(update(&mut s, m, mag, &P), MotionEvent::None);
        assert_eq!(update(&mut s, m, mag, &P), MotionEvent::BecameParked);
        assert!(!s.is_moving());
    }

    #[test]
    fn parked_park_reports_none() {
        let mut s = state();
        let (m, mag) = idle(1000);
        for _ in 0..10 {
            assert_eq!(update(&mut s, m, mag, &P), MotionEvent::None);
        }
    }

    #[test]
    fn magic_preserved_across_updates() {
        let mut s = state();
        update(&mut s, active(), 1000, &P);
        assert_eq!(s.magic, STATE_MAGIC);
    }
}
