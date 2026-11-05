//! Wheelchair location tracker: motion-gated reporting on ESP32-C3 +
//! SIM7600CE-T + LIS2DH.
//!
//! Every code path ends in deep sleep; the wake cause decides what happens.
//! Issues #1-#6 of the tracker repo are fixed by construction here - see
//! `lib.rs` for the mapping.
//!
//! Hardware status (2026-10-29): boot, RTC-state persistence, exact mono
//! restoration and the sleep/wake loop are verified on a real C3 with no
//! peripherals attached. The modem/I2C/INT1 paths remain bench-pending - see
//! `firmware_rs/README.md` and `docs/hardware.md`.

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Event, Input, InputConfig, Level, Output, OutputConfig, Pull, WakeupConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::rtc_cntl::sleep::{LowPower, RtcSleepConfig};
use esp_hal::rtc_cntl::{WakeupSource, wakeup_cause};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use heapless::String;
use static_cell::StaticCell;

use firmware_rs::config as cfg;
use firmware_rs::config::ReportVia;
use firmware_rs::uplink;
use firmware_rs::motion::{self, MotionEvent, MotionParams, WakeMotion};
use firmware_rs::state::PersistState;
use firmware_rs::{lis2dh, sim7600};

esp_bootloader_esp_idf::esp_app_desc!();

/// Persisted state in RTC-fast RAM. Deep sleep resets the CPU and main RAM;
/// this static is the only thing that survives (issue #1).
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static mut PERSIST: PersistState = PersistState::ZEROED;

/// The modem's UART singleton, parked so `report` can borrow it every wake.
static MODEM_UART: StaticCell<Uart<'static, esp_hal::Async>> = StaticCell::new();

const MODEM_SYNC_DELAY_MS: u32 = 500;
/// The module needs seconds to boot; keep the probe loop honest.
const MODEM_BOOT_ATTEMPTS: u8 = 24;
const MIN_SLEEP_MS: u64 = 2000;
/// Retry back-off after a failed report, before the next cadence step.
const RETRY_AFTER_FAIL_MS: u64 = 60_000;

#[esp_rtos::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    // 80 MHz is sufficient for the short motion-only path and avoids paying
    // the 160 MHz active-current cost on every GPIO wake.
    let peripherals = esp_hal::init(
        esp_hal::Config::default().with_cpu_clock(CpuClock::default()),
    );

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    let mut lp = LowPower::new(peripherals.LPWR);

    // PWRKEY gate: idle LOW, pulse HIGH. The level is set here, at
    // construction - never written before the pin exists (issue #5).
    let mut pwrkey_gate = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());

    // LIS2DH INT1: active-high threshold output. Pull-down so the pad is
    // held at the non-wake level during sleep (a floating pad wakes
    // immediately and every time).
    let mut imu_int = Input::new(
        peripherals.GPIO0,
        InputConfig::default().with_pull(Pull::Down),
    );

    let mut i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(peripherals.GPIO8)
        .with_scl(peripherals.GPIO9);

    let modem_uart = MODEM_UART.init(
        Uart::new(peripherals.UART1, UartConfig::default().with_baudrate(115_200))
            .unwrap()
            .with_rx(peripherals.GPIO5)
            .with_tx(peripherals.GPIO4)
            .into_async(),
    );

    let state: &'static mut PersistState = unsafe { &mut *(&raw mut PERSIST) };
    if !state.is_valid() {
        state.reset();
        state.next_report_ms = state.mono_ms + cfg::POLL_MS as u64;
        state.seal();
    }

    let params = MotionParams {
        enter_mg: cfg::MOVE_ENTER_MG,
        exit_mg: cfg::MOVE_EXIT_MG,
        samples: cfg::HYSTERESIS_SAMPLES,
    };
    {
        let mut imu = lis2dh::Lis2dh::new(&mut i2c, cfg::IMU_ADDR);
        if matches!(imu.probe(), Ok(true)) {
            let _ = imu.configure(cfg::IMU_THRESHOLD, cfg::IMU_DURATION);
        } else {
            esp_println::println!("imu=fault");
        }
    }
    arm_int1_wake(&mut imu_int);

    let cause = wakeup_cause();
    if cause.contains(WakeupSource::Gpio) {
        esp_println::println!("wake=GPIO");
        on_motion_wake(state, &mut i2c, &mut imu_int, &params);
    } else if cause.contains(WakeupSource::Timer) {
        esp_println::println!("wake=Timer mono={}ms", state.mono_ms);
        on_timer_wake(
            state,
            &mut i2c,
            &mut imu_int,
            &params,
            &mut pwrkey_gate,
            modem_uart,
        )
        .await;
    } else {
        esp_println::println!("wake=PowerOn state_reset={}", state.mono_ms == 0);
    }
    // Power-on reset (empty cause): the init above already set the first
    // report target.

    arm_and_sleep(&mut lp, &mut imu_int, state, sleep_duration_ms(state)).await
}

/// Motion-only wake: classify, bill one poll period, never touch the modem.
fn on_motion_wake(
    state: &mut PersistState,
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    imu_int: &mut Input<'_>,
    params: &MotionParams,
) {
    // Sleep length is unknown on this path; bill one poll period as the
    // lower bound, so cadence math stays conservative.
    state.mono_ms += cfg::POLL_MS as u64;

    let event = classify_motion(state, i2c, imu_int, params);
    if !state.is_settled() {
        state.set_settled(true);
        state.next_report_ms = if state.is_moving() {
            state.mono_ms + MIN_SLEEP_MS
        } else {
            state.mono_ms + cfg::SEND_IDLE_MS as u64
        };
    }
    if event != MotionEvent::None {
        // Transition: report as soon as the sleep floor allows.
        state.next_report_ms = state.mono_ms + MIN_SLEEP_MS;
    }
    state.seal();
}

/// Timer wake: the heartbeat. Reports when due, re-arms otherwise.
async fn on_timer_wake(
    state: &mut PersistState,
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    imu_int: &mut Input<'_>,
    params: &MotionParams,
    gate: &mut Output<'_>,
    modem_uart: &'static mut Uart<'static, esp_hal::Async>,
) {
    // Timer wakes are exact: the armed deadline was `sleep_target_ms`.
    state.mono_ms = state.sleep_target_ms.max(state.mono_ms);

    let _ = classify_motion(state, i2c, imu_int, params);

    if !state.is_settled() {
        state.set_settled(true);
        state.seal();
    }

    let due = state.mono_ms >= state.next_report_ms;
    esp_println::println!(
        "timer: mono={}ms next={}ms due={}",
        state.mono_ms,
        state.next_report_ms,
        due
    );
    if due {
        report(state, gate, modem_uart, "tick").await;
    }
}

/// Reads the IMU and advances the hysteresis. Returns a transition event.
fn classify_motion(
    state: &mut PersistState,
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    imu_int: &mut Input<'_>,
    params: &MotionParams,
) -> MotionEvent {
    let int1_latched = imu_int.is_interrupt_set();
    let mut imu = lis2dh::Lis2dh::new(i2c, cfg::IMU_ADDR);
    let magnitude_mg = imu.magnitude(200);
    if magnitude_mg < 0 {
        // A dead/unwired IMU must not be interpreted as PARKED. Treat the
        // device as active so the timer path continues acquiring fixes.
        esp_println::println!("imu=read-fault");
        state.set_moving(true);
    }
    let polled_moving = magnitude_mg > 0 && (magnitude_mg - 1000).abs() > cfg::MOVE_ENTER_MG;

    let event = motion::update(
        state,
        WakeMotion {
            int1_latched,
            polled_moving,
        },
        magnitude_mg,
        params,
    );
    // Reading INT1_SRC clears the latch (ST side effect), so the next
    // classification starts clean.
    let _ = imu.clear_interrupt();
    state.seal();
    event
}

/// Full report sequence: modem on, GNSS when needed, send, modem off.
async fn report(
    state: &mut PersistState,
    gate: &mut Output<'_>,
    modem_uart: &'static mut Uart<'static, esp_hal::Async>,
    why: &str,
) {
    let awake_started = Instant::now();
    let mut sim = sim7600::Sim7600::new(modem_uart, Duration::from_millis(1500), 2);

    let modem_on = ensure_modem_on(gate, &mut sim).await && sim.bring_up(Some(cfg::APN)).await;
    esp_println::println!("report({}): modem_on={}", why, modem_on);
    if !modem_on {
        advance_awake(state, awake_started);
        state.next_report_ms = state.mono_ms + RETRY_AFTER_FAIL_MS;
        state.seal();
        return;
    }

    // Fresh fix only while moving; a parked chair re-sends its cache.
    let fix_ok = if fix_fresh(state) {
        true
    } else if state.is_moving() || !state.has_fix() {
        acquire_fix(state, &mut sim).await
    } else {
        false
    };

    if !fix_ok && !state.has_fix() {
        advance_awake(state, awake_started);
        state.next_report_ms = state.mono_ms + RETRY_AFTER_FAIL_MS;
        state.seal();
        modem_off(&mut sim, gate).await;
        return;
    }

    advance_awake(state, awake_started);
    let sent = send_report(state, &mut sim, why).await;
    esp_println::println!("report: sent={}", sent);

    // Cadence advances on a successful send; failures retry sooner.
    state.next_report_ms = state.mono_ms
        + if sent {
            (if state.is_moving() {
                cfg::SEND_MOVING_MS
            } else {
                cfg::SEND_IDLE_MS
            }) as u64
        } else {
            RETRY_AFTER_FAIL_MS
        };
    state.seal();

    modem_off(&mut sim, gate).await;
}

/// GNSS acquisition: powered poll until a valid fix or the deadline.
async fn acquire_fix(state: &mut PersistState, sim: &mut sim7600::Sim7600<'_>) -> bool {
    if !sim.gnss_on().await {
        return false;
    }
    let deadline = Instant::now() + Duration::from_millis(cfg::FIX_DEADLINE_MS as u64);
    let mut got = None;
    while Instant::now() < deadline {
        if let Some(fix) = sim.position().await {
            if fix.valid() {
                got = Some(fix);
                break;
            }
        }
        Timer::after_millis(2000).await;
    }
    sim.gnss_off(4000).await;

    let Some(fix) = got else {
        return false;
    };
    state.lat_e7 = fix.lat_e7;
    state.lon_e7 = fix.lon_e7;
    state.sats = fix.sats;
    state.unix_fix = fix.unix_time;
    state.fix_mono_ms = state.mono_ms;
    state.set_had_fix(true);
    state.seal();
    true
}

fn advance_awake(state: &mut PersistState, started: Instant) {
    let elapsed = Instant::now().saturating_duration_since(started).as_millis();
    state.mono_ms = state.mono_ms.saturating_add(elapsed);
}

/// Sends via the configured channel (SMS default, HTTP opt-in).
async fn send_report(
    state: &mut PersistState,
    sim: &mut sim7600::Sim7600<'_>,
    why: &str,
) -> bool {
    let age_ms = state.fix_age_ms(state.mono_ms);
    let age_s = (age_ms / 1000) as u32;
    let stale = age_ms > cfg::FIX_STALE_MS as u64;

    let mut body = String::<160>::new();
    let _ = core::fmt::Write::write_fmt(
        &mut body,
        format_args!(
            "{}{} {},{} s{} a{}s{} {}",
            if state.is_moving() { "M" } else { "S" },
            if stale { "*" } else { "" },
            deg_str(state.lat_e7),
            deg_str(state.lon_e7),
            state.sats,
            age_s,
            if stale { "(stale)" } else { "" },
            why
        ),
    );

    match cfg::REPORT_VIA {
        ReportVia::Sms if !cfg::OWNER.is_empty() => {
            sim.sms_send(cfg::OWNER, body.as_str()).await.is_ok()
        }
        ReportVia::Sms => {
            esp_println::println!("sms=disabled owner_not_configured");
            false
        }
        ReportVia::Http => {
            // One atomic point keeps the location, fix age, motion state and
            // GNSS timestamp together. The server must not infer freshness
            // from its receive time.
            let mut loc_body = String::<256>::new();
            let _ = core::fmt::Write::write_fmt(
                &mut loc_body,
                format_args!(
                    "{{\"value\":\"{}\",\"lat\":{},\"lon\":{},\"created_at\":{}}}",
                    body.as_str(), deg_str(state.lat_e7), deg_str(state.lon_e7), state.unix_fix
                ),
            );

            // The key goes in the query string because AT+HTTPPARA cannot set
            // custom headers (see uplink.rs).
            let mut url = String::<320>::new();
            let _ = core::fmt::Write::write_fmt(
                &mut url,
                format_args!(
                    "http://{}/api/v2/{}/feeds/{}/data?X-AIO-Key={}",
                    cfg::SERVER_HOST,
                    cfg::AIO_USER,
                    cfg::AIO_FEED_LOCATION,
                    cfg::AIO_KEY
                ),
            );

            match uplink::post_json(sim, url.as_str(), loc_body.as_bytes()).await {
                Ok(status) => {
                    esp_println::println!("http={}", status);
                    true
                }
                Err(e) => {
                    esp_println::println!("http={:?}", e);
                    false
                }
            }
        }
    }
}

fn fix_fresh(state: &PersistState) -> bool {
    state.has_fix() && state.fix_age_ms(state.mono_ms) <= cfg::FIX_VALID_MS as u64
}

/// Degrees x 1e7 -> "-106.8452300" style fixed-point string. No floats.
fn deg_str(e7: i32) -> String<16> {
    let mut out = String::new();
    if e7 < 0 {
        let _ = out.push('-');
    }
    let a = e7.unsigned_abs();
    let _ = core::fmt::Write::write_fmt(&mut out, format_args!("{}.", a / 10_000_000));
    let _ = core::fmt::Write::write_fmt(&mut out, format_args!("{:07}", a % 10_000_000));
    out
}

/// Idempotent power-on (issue #4): probe first, pulse PWRKEY only if silent.
async fn ensure_modem_on(gate: &mut Output<'_>, sim: &mut sim7600::Sim7600<'_>) -> bool {
    if sim.answers().await {
        return true;
    }
    pulse_pwrkey(gate).await;
    sim.sync(MODEM_BOOT_ATTEMPTS, MODEM_SYNC_DELAY_MS).await
}

/// PWRKEY pulse through the N-MOSFET gate: HIGH 1500 ms, then idle LOW.
async fn pulse_pwrkey(gate: &mut Output<'_>) {
    gate.set_level(Level::High);
    Timer::after_millis(cfg::MODEM_PWRKEY_HOLD_MS as u64).await;
    gate.set_level(Level::Low);
    // The module needs a moment before the first AT gets through.
    Timer::after_millis(2000).await;
}

/// Powers the module down fully for sleep. `AT+CPOF` is preferred; the
/// PWRKEY fallback covers builds without it (bench item).
async fn modem_off(sim: &mut sim7600::Sim7600<'_>, gate: &mut Output<'_>) {
    if !sim.ok("AT+CPOF").await {
        pulse_pwrkey(gate).await;
    }
    Timer::after_millis(2000).await;
}

fn sleep_duration_ms(state: &PersistState) -> u64 {
    state
        .next_report_ms
        .saturating_sub(state.mono_ms)
        .clamp(MIN_SLEEP_MS, cfg::WATCHDOG_MS as u64)
}

/// Arms the INT1 low-power wake path. Persistent: the config outlives the
/// driver, and a deep-sleep wake starts with no wakeup sources armed.
fn arm_int1_wake(imu_int: &mut Input<'_>) {
    imu_int.listen(Event::HighLevel);
    let _ = imu_int.apply_wakeup_config(&WakeupConfig::default().with_low_power_path(true));
}

/// Arms the timer deadline and enters deep sleep. Does not return.
async fn arm_and_sleep(
    lp: &mut LowPower<'_>,
    imu_int: &mut Input<'_>,
    state: &'static mut PersistState,
    sleep_ms: u64,
) -> ! {
    state.sleep_target_ms = state.mono_ms + sleep_ms;
    state.seal();
    esp_println::println!("sleep {}ms", sleep_ms);
    arm_int1_wake(imu_int);
    lp.set_wakeup_deadline(
        esp_hal::time::Instant::now() + esp_hal::time::Duration::from_millis(sleep_ms),
    );
    lp.sleep_deep(RtcSleepConfig::default())
}
