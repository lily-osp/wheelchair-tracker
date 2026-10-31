# wheelchair-tracker

Motion-gated location tracker: ESP32-C3 Super Mini + SIM7600CE-T + LIS2DH.

**Rust, `no_std`, esp-hal** — see `firmware_rs/`. The SIM7600 provides **both**
GNSS and internet: the default uplink is SMS; an opt-in HTTP channel runs PPP
over the modem's AT UART. The ESP32's own WiFi/BT radio is unused.

**See `firmware_rs/README.md` for the full documentation** — build, flashing,
pin map, power budget, runtime model, and the bench checklist.

The earlier MicroPython and Arduino C++ prototypes were removed; everything
they proved about this hardware (modem command quirks, wake behaviour, power
budget) is baked into `firmware_rs/` and [`docs/`](docs/).

## The finding that shapes the whole design

The LIS2DH does **not** measure position. It measures acceleration, and position
would need two integrations, which amplifies MEMS bias without bound (~10 mg
bias → ~180 m error after one minute, ~4.5 km after five). So the accelerometer
has one job here:

> **Decide whether the chair is moving, so the tracker knows when to pay for a
> GNSS acquisition.**

For real dead reckoning you would need a wheel/axle encoder, not an IMU.

## Power budget — the numbers that shape the design

Modelled from the SIM7600's datasheet idle/GNSS figures and the C3's measured
deep-sleep current:

```
GNSS cold start:  35 s × 120 mA × 3.3 V ≈ 13.9 J per fix
C3 deep sleep:                       ≈ 0.4 J per full day
```

**One cold fix costs roughly 35 days of sleep** — a modelled ratio, not a
measured one. Every design decision follows from it:

- Never acquire a GNSS fix while parked. Re-send the cached fix with an age
  marker instead — a parked chair's position has not changed, so a stale
  position that is honestly labelled beats a fresh one never acquired.
- A 5 s send interval while moving is not viable on SMS (720 pings/hour). The
  prototype used 60 s moving / 15 min parked. **These are guesses, not
  calculations** — check Telkomsel's per-SMS tariff before committing.
- Desolder the C3 SuperMini's onboard LED. It draws 1.3–2 mA ≈ 158 J/day,
  roughly **400× the entire sleep budget**, and it will dominate runtime no
  matter how good the gating is. It also frees GPIO8, the recommended I²C pin.

## GPIO wake: direct, no polling workaround

The prototype woke on a 5 s timer and polled the accelerometer, because
**GPIO wake from deep sleep is unreliable on the C3 under MicroPython**
(micropython#16839, #11787).

`firmware_rs/` removes that workaround: the HAL's `Input::listen(Event)` +
`apply_wakeup_config(&WakeupConfig::default().with_low_power_path(true))` arms
INT1 for deep-sleep wake, with a pull-down holding the pad at the non-wake
level.

The shipping design is:

- **LIS2DH INT1 → C3 GPIO0**, waking deep sleep directly.
- **A long timer as the watchdog**, so the device still reports even if the
  accelerometer dies or its threshold never fires. A tracker that can only wake
  on motion is a tracker that goes silent the day the sensor fails.
- Idle state = deep sleep with ~0.5–1 µA, instead of polling every 5 s.

## Architecture

```
        ┌──────────────┐  I2C    ┌────────────┐
        │  ESP32-C3    │◄───────►│  LIS2DH    │ ~3 µA
        │  SuperMini   │  INT1 ──┼──► GPIO0   │ wakes deep sleep
        └──────┬───────┘         └────────────┘
               │ UART
        ┌──────▼───────┐   own 4.0 V / 2 A supply
        │  SIM7600CE-T │   GNSS + internet: SMS default,
        └──────┬───────┘   HTTP over PPP opt-in
               │
        ┌──────▼────────────────────────────────────┐
        │  Li-ion → boost 4.0 V (modem)            │
        │  Li-ion → 3.3 V regulator (C3 + LIS2DH)  │
        └───────────────────────────────────────────┘
```

## Layout

```
wheelchair-tracker/
├── firmware_rs/            the firmware
│   ├── src/bin/main.rs     wake-path state machine
│   ├── src/*.rs            AT/GNSS/motion/state, host-tested
│   └── README.md           ← build, flashing, pins, bench checklist
├── docs/
│   ├── hardware.md         pin map + bench checklist
│   └── modem-quirks.md     SIM7600 command quirks, third-party-lib warnings
├── flake.nix               pinned dev shell + `nix run .#flash`
└── README.md
```

The modem's command quirks are a whole section of this project's history — see
[`docs/modem-quirks.md`](docs/modem-quirks.md) before writing any AT transport
for this module.

## Status

All parsing/classification/validation logic is verified by the host test suite
(39 tests, `cargo test` in `firmware_rs/`). First hardware smoke test passed
(2026-10-29, ESP32-C3 SuperMini, no peripherals): flash, boot, esp-hal/esp-rtos
init, RTC-fast state persistence, exact `mono_ms` restoration on timer wake,
and a stable wake → classify → report → deep-sleep loop — no panics.

**Still bench-pending:** the SIM7600 UART transport on the ESP32 (GPIO4/5),
LIS2DH I2C reads, INT1 GPIO wake, `AT+CPOF` recovery, and measured sleep
current — everything that needs the peripherals wired. `docs/hardware.md`
carries the bench checklist.

The modem was verified over USB on 2026-10-31: identity, SIM, signal,
operator (Telkomsel), registration, APN `cmnet`, IP `10.0.68.103`, cellular
DNS, SMS round trip, GNSS fix (real coordinates), and PPP dial + HTTP POST to
Adafruit IO (200 OK). Those findings are encoded in
`firmware_rs/src/at.rs` and `sim7600.rs`. The HTTP uplink now targets
Adafruit IO feeds `location` and `time` (key/username/feed names live in
`firmware_rs/adafruit.toml`, not in source).

**The full tracker path has not run with a LIS2DH attached yet.** Run
[`docs/hardware.md`](docs/hardware.md)'s bench checklist in order before
trusting anything. (Board-only smoke test passed — see above.)

## Privacy

A device that continuously reports where a person is, over a public cellular
network, is a tracking device whether or not it is used that way. For a
disability aid the person should know it exists, hold the SIM, and be able to
switch it off physically — not just via a setting in this firmware.