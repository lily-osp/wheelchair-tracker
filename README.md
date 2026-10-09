# wheelchair-tracker

Motion-gated location tracker: ESP32-C3 Super Mini + SIM7600CE-T + LIS2DH.

Target is **Arduino C++**, in `wheelchair_tracker/`. The MicroPython prototype
under `poc/` was for fast bring-up against real hardware and is not what ships.

**See `wheelchair_tracker/README.md` for the full firmware documentation** —
build flags, flashing, pin configuration, power budget, and the modem's firmware
quirks.

The two things that will waste your time if you skip them:

- Build with `--fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc`. The default compiles
  `ARDUINO_USB_CDC_ON_BOOT=0`, and the app's `Serial` output then goes nowhere
  while the boot ROM log still prints — it looks exactly like a dead board.
- The sketch folder name must match the `.ino` filename.

## The finding that shapes the whole design

The LIS2DH does **not** measure position. It measures acceleration, and position
would need two integrations, which amplifies MEMS bias without bound (~10 mg
bias → ~180 m error after one minute, ~4.5 km after five). So the accelerometer
has one job here:

> **Decide whether the chair is moving, so the tracker knows when to pay for a
> GNSS acquisition.**

For real dead reckoning you would need a wheel/axle encoder, not an IMU.

## Power budget — the decisive numbers

```
GNSS cold start:  35 s × 120 mA × 3.3 V ≈ 13.9 J per fix
C3 deep sleep:                       ≈ 0.4 J per full day
```

**One cold fix costs roughly 35 days of sleep.** Every design decision follows
from that:

- Never acquire a GNSS fix while parked. Re-send the cached fix with an age
  marker instead — a parked chair's position has not changed, so a stale
  position that is honestly labelled beats a fresh one never acquired.
- A 5 s send interval while moving is not viable on SMS (720 pings/hour). The
  prototype used 60 s moving / 15 min parked. **These are guesses, not
  calculations** — check Telkomsel's per-SMS tariff before committing.
- Desolder the C3 SuperMini's onboard LED. It draws 1.3–2 mA ≈ 158 J/day,
  roughly **400× the entire sleep budget**, and it will dominate runtime no
  matter how good the gating is. It also frees GPIO8, the recommended I²C pin.

## GPIO wake works in Arduino — use it

The prototype woke on a 5 s timer and polled the accelerometer, because
**GPIO wake from deep sleep is unreliable on the C3 under MicroPython**
(micropython#16839, #11787).

In Arduino that problem disappears. On the C3, `EXT0`/`EXT1` are unavailable, but
`esp_deep_sleep_enable_gpio_wakeup()` works on RTC pins GPIO0–GPIO5:

```cpp
esp_deep_sleep_enable_gpio_wakeup(BIT(0), ESP_GPIO_WAKEUP_GPIO_HIGH);
esp_deep_sleep_start();
```

Wake reason is reported as `0x05` (`ESP_SLEEP_WAKEUP_GPIO`), not the EXT0/EXT1
values you may expect from ESP32 documentation — worth a sanity check on first
bring-up.

So the shipping design is:

- **LIS2DH INT1 → C3 GPIO0**, waking deep sleep directly.
- **A long timer as the watchdog**, so the device still reports even if the
  accelerometer dies or its threshold never fires. A tracker that can only wake
  on motion is a tracker that goes silent the day the sensor fails.
- Idle state = deep sleep with ~0.5–1 µA, instead of polling every 5 s.

This is the clearest argument for the C++ port: it is not just a language
preference, it removes a workaround and cuts idle current further.

## Architecture

```
        ┌──────────────┐  I2C    ┌────────────┐
        │  ESP32-C3    │◄───────►│  LIS2DH    │ ~3 µA
        │  SuperMini   │  INT1 ──┼──► GPIO0   │ wakes deep sleep
        └──────┬───────┘         └────────────┘
               │ UART
        ┌──────▼───────┐   own 4.0 V / 2 A supply
        │  SIM7600CE-T │   SMS uplink only (no TCP on this firmware)
        └──────┬───────┘
               │
        ┌──────▼────────────────────────────────────┐
        │  Li-ion → boost 4.0 V (modem)            │
        │  Li-ion → 3.3 V regulator (C3 + LIS2DH)  │
        └───────────────────────────────────────────┘
```

## Layout

```
wheelchair-tracker/
├── wheelchair_tracker/   Arduino C++ — what ships
│   ├── wheelchair_tracker.ino
│   ├── Sim7600At.{h,cpp}  AT transport, ported from the verified prototype
│   ├── lis2dh.{h,cpp}     I²C accelerometer, threshold interrupt
│   ├── Nmea.{h,cpp}       NMEA parser with checksum validation
│   └── README.md          ← build, flashing, pins, power, quirks
├── test/                 host suite (53 assertions) + Arduino shims
├── poc/                  MicroPython prototype — verified reference
├── hardware.md           bench checklist
└── README.md
```

The MicroPython prototype is not throwaway. Its command set was verified against
the live module and the firmware's quirks are recorded there — port those
findings, not just the code:

- **No AT data channels.** `AT+CIPSOCK`, `AT+CIPSTART`, `AT+CIPSTATUS`,
  `AT+CIFSR`, all `AT+CHTTP*`, all `AT+CMQ*` return `ERROR`. SMS and
  `AT+CDNSGIP` work. So no MQTT-over-cellular on this module as it stands.
- **GNSS is `AT+CGPS` / `AT+CGPSNMEAPORTCFG`, not `AT+CGNSPWR`.** Every standard
  GNSS form errors on this firmware.
- **`AT+CGPSNMEAPORTCFG=3` is a trap** — it routes NMEA onto the AT port and
  wedges the command channel. Use port 1.
- **With `ATE0`, action-form commands return bare values** — `AT+CGMM` →
  `SIM7600CE-T`, no `+CGMM:` prefix, with leading blank lines. A parser that
  only looks for prefixed lines silently returns nothing for identity.
- **`+CPIN`/`+CEREG` must not be treated as unsolicited result codes**, or the
  reply to the query that asked for them gets discarded.
- **`AT+CIFSR` is broken**; use `AT+CGPADDR`.

## On third-party libraries

TinyGSM and the community SIM7600 libraries assume a working `AT+CIPSOCK`/
`AT+CIPSTART`. On this firmware those commands fail, so their network paths are
dead on arrival here. The AT transport in `firmware/` is ported from the
prototype specifically because it was verified against this module's actual
behaviour.

## Status

Prototype verified by simulation (fake I²C register model, fake IMU) and against
the live modem over USB: identity, SIM, signal, operator, registration, APN, IP,
cellular DNS, SMS list/read/send, GNSS on/off, capability probe, input
rejection.

**Neither the firmware nor a LIS2DH has run on hardware yet.** The modem was
offline during development. See `hardware.md` for the bench checklist — run it in
order before trusting anything.

## Privacy

A device that continuously reports where a person is, over a public cellular
network, is a tracking device whether or not it is used that way. For a
disability aid the person should know it exists, hold the SIM, and be able to
switch it off physically — not just via a setting in this firmware.