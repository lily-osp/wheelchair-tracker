# wheelchair_tracker — Arduino C++ firmware

Motion-gated location tracker: **ESP32-C3 SuperMini + SIM7600CE-T + LIS2DH**.

The accelerometer does **not** measure position — it decides whether the chair
is moving, so the tracker only pays for a GNSS acquisition when it is. That gate
is the whole power budget:

```
GNSS cold start:  35 s x 120 mA x 3.3 V = 13.9 J per fix
C3 deep sleep:                       = 0.4 J per full day
```

One cold fix costs roughly **35 days of sleep**. Every design decision follows
from that ratio.

---

## 1. Build

**`CDCOnBoot=cdc` is mandatory.** The Arduino default is `Disabled`, which
compiles with `-DARDUINO_USB_CDC_ON_BOOT=0`. With that, `Serial` is not the USB
CDC at all: the ESP32 boot-ROM log still prints over USB (it is raw ROM output)
but every `Serial.print` from the app goes nowhere — indistinguishable from a
dead board.

```bash
cd ~/project/microcontrollers/03_projects/esp32c3/wheelchair-tracker

# Sketch folder name MUST match the .ino filename: wheelchair_tracker/
arduino-cli compile \
  --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc" \
  --warnings all \
  --build-path /tmp/wctr_build \
  wheelchair_tracker/
```

Verified on hardware:

| | |
|---|---|
| Core | esp32 3.3.11 |
| Flash | 361184 B (27% of 1.3 MB) |
| RAM | 17136 B (5%) |
| Warnings | none, with `--warnings all` |

### Host tests

```bash
sh test/run.sh          # 53 assertions, no board needed
```

30 for the AT transport, 23 for LIS2DH + NMEA. These compile the real `.cpp`
files against minimal Arduino shims in `test/shim/` with a fake I²C register
model and a scripted clock. The `.ino` is **not** covered — it needs the real
core.

---

## 2. Flashing

```bash
ESP=~/.espressif/python_env/idf6.0_py3.13_env/bin/esptool
B=/tmp/wctr_build

$ESP --chip esp32c3 --port /dev/ttyACM0 --baud 460800 write-flash -z \
  --flash-mode dio --flash-freq 80m \
  0x0     $B/wheelchair_tracker.ino.bootloader.bin \
  0x8000  $B/wheelchair_tracker.ino.partitions.bin \
  0xe000  $B/boot_app0.bin \
  0x10000 $B/wheelchair_tracker.ino.bin
```

Use the **default 4 MB partition scheme.** The 8 MB scheme produces a partition
table that does not fit a 4 MB chip, and the board boot-loops:

```
E (24) flash_parts: partition 3 invalid - offset 0x340000 size 0x330000
    exceeds flash chip size 0x400000
E (26) boot: Failed to verify partition table
```

### Getting a reset out of download mode

`write-flash` leaves the chip in the ROM stub (`boot:0x5`, `waiting for
download`). On the C3's USB-Serial/JTAG peripheral, **RTS drives BOOT (IO0) and
DTR drives EN**:

```python
s.dtr = True    # EN low   -> reset
s.dtr = False   # EN high  -> run from flash, IO0 high = normal boot
```

RTS must be released. Do **not** run `esptool` again after flashing or you knock
it straight back into download mode.

### Reading serial reliably

Two traps, both of which cost real debugging time:

1. **DTR must be asserted** before the app transmits.
2. **Re-open the port in a loop.** A chip reset re-enumerates USB, so a single
   long-lived file descriptor goes stale and silently drops everything after the
   reset. That is why naive captures see the boot ROM and nothing else.

```python
import serial, time, os
P = "/dev/ttyACM0"
s = serial.Serial(P, 115200, timeout=0.05)
s.dtr, s.rts = True, False
s.dtr = False                  # boot from flash
time.sleep(0.4)
while True:
    c = s.read(256)
    if c: print(c.decode("utf-8", "replace"), end="", flush=True)
```

A udev rule keeps the CDC node in your `dialout` group:

```
# /etc/udev/rules.d/99-esp32-c3.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", MODE="0660", GROUP="dialout"
```

### Expected boot output

With nothing attached:

```
pulsing PWRKEY
modem silent - check PWRKEY and the 4.0 V supply
```

With a modem and IMU:

```
pulsing PWRKEY
SIMCOM INCORPORATED SIM7600CE-T fw=LE20B05SIM7600M22
sms=1 dns=1 gnss=1 tcp=0 http=0 mqtt=0
LIS2DH ready, threshold 0x08 (0.5 g)
```

`sms=1` is the one that matters — without it the tracker cannot report.

---

## 3. Pin configuration

Defined at the top of `wheelchair_tracker.ino`:

```cpp
static const int MODEM_TX = 4;        // C3 TX  -> modem RXD
static const int MODEM_RX = 5;        // C3 RX  <- modem TXD
static const int PWRKEY_GATE = 6;     // N-MOSFET gate, never PWRKEY direct
static const int IMU_SDA = 8;         // onboard LED must be removed
static const int IMU_SCL = 9;
static const int IMU_INT1 = 0;        // RTC pin 0-5 wakes deep sleep on C3
static const uint8_t IMU_ADDR = 0x19; // 0x18 if SDO/SA0 is pulled low
```

| C3 pin | Connects to | Notes |
|---|---|---|
| GPIO0 | LIS2DH INT1 | must be an RTC pin (0–5) for deep-sleep wake |
| GPIO4 | modem RXD | UART1 TX, 115200 8N1 |
| GPIO5 | modem TXD | UART1 RX |
| GPIO6 | PWRKEY gate | N-MOSFET gate — **not** PWRKEY directly |
| GPIO8 | LIS2DH SDA | onboard LED sits here, remove it |
| GPIO9 | LIS2DH SCL | |
| 3V3 | LIS2DH VCC | fine, the part draws µA |
| 4.0 V | modem VBAT | via boost — **never** the 3V3 rail |
| GND | everything | mandatory common ground |

**GPIO18/19 are unusable.** They are the USB Serial/JTAG pair, and USB is the
only programming path on this board.

C3 SuperMini pinouts vary between sellers — verify against your silkscreen.

### Wake polarity

LIS2DH INT1 is **active high**: driven low when idle, rising when the threshold
is crossed (polarity is settable via `CTRL_REG3`/`INTx_POL`, left at its reset
value here). The firmware wakes on `ESP_GPIO_WAKEUP_GPIO_HIGH`.

**Unverified** — no LIS2DH attached yet. If motion never wakes the chip, flip
this polarity first.

### I²C address

`0x19` with SDO/SA0 high (usual breakout wiring), `0x18` with it pulled low.
`WHO_AM_I` must read `0x32`; `0x33`/`0x2A` are accepted as clones.

---

## 4. Power

| Item | Current | Note |
|---|---|---|
| C3 deep sleep | ~5 µA | the design target |
| C3 active | ~30 ms | during a GNSS read cycle |
| LIS2DH low-power | ~3 µA | 10 Hz ODR, threshold interrupt armed |
| SIM7600 idle | ~10 mA | **the largest remaining drain** |
| SIM7600 GNSS acquisition | ~120 mA | **~35 s** |
| SIM7600 TX burst | up to 2 A | milliseconds, but must not brown out |

### Remove the onboard LED — do this first

The C3 SuperMini's LED on GPIO8 draws 1.3–2 mA:

```
2 mA x 3.3 V x 24 h = 158 J/day   vs   ~0.4 J/day of deep sleep
```

That is roughly **400× the entire sleep budget**. It will dominate runtime no
matter how good the gating is. Desolder it, or remove the series resistor and
drive the pin appropriately. It also frees GPIO8, the recommended I²C pin.

### Supply

The modem **cannot** be powered from the C3's 3.3 V rail. VBAT needs 3.4–4.3 V
and the radio pulls close to 2 A in transmit bursts. On 3V3 it browns out
mid-burst, drops off the network, and can fail USB enumeration outright.

Li-ion cell, split:

- **C3 + LIS2DH** ← cell's 3.7 V direct
- **modem** ← boost to 4.0 V that holds under a 2 A burst

A marginal converter is the most common cause of "works on the bench, fails in
the chair" — brownouts look like flaky registration, not like power faults.

The modem's ~10 mA idle is currently the biggest remaining drain. Cutting VBAT
needs a load switch plus PWRKEY sequencing and re-registration time, and is
**not implemented**.

---

## 5. Firmware behaviour

### Motion gate

`updateMotion()` samples the accelerometer and applies hysteresis:

```
MOVE_ENTER_MG = 250    must exceed this, 3 consecutive samples, to be "moving"
MOVE_EXIT_MG  = 120    must fall below this, 3 consecutive, to be "parked"
HYSTERESIS_SAMPLES = 3
```

Hysteresis is not optional. Without it a chair stopped at a kerb flips state
every few seconds and fires an SMS each time.

Two sources are OR'd: the accelerometer's own latched threshold interrupt, and a
magnitude cross-check. The cross-check means a missed interrupt degrades
instead of silently reading as "parked".

Magnitude, not per-axis deviation: gravity fixes the vector length at 1 g
regardless of how the chair is tilted.

### Reporting

| State | Interval |
|---|---|
| Moving | 60 s |
| Parked | 15 min heartbeat |
| No fix ever | sends nothing |
| Fix older than 10 min | re-sent, labelled `stale` |

SMS format, e.g. `M -6.23506,106.84561 s8 a12s` — `M`/`S` for moving/parked,
`*` prefix and `(stale)` when the fix is old, then coordinates, satellites and
fix age.

**The 60 s interval is a plausibility guess, not a calculation.** Check Telkomsel's
per-SMS tariff and recalculate. 5 s while moving is not viable on SMS — that is
720 pings/hour if the chair keeps moving.

### Deep sleep and persistence

Idle state is deep sleep with ~0.5–1 µA. Wake sources:

- **LIS2DH INT1** on GPIO0
- **a timer**, capped at `POLL_MS`, as the watchdog path

The timer is not optional: a tracker that can only wake on motion goes silent the
day the accelerometer fails. `WATCHDOG_MS` (1 h) forces a report regardless.

`EXT0`/`EXT1` do not exist on the C3. GPIO wake via
`esp_deep_sleep_enable_gpio_wakeup()` works on RTC pins GPIO0–5, and reports
`ESP_SLEEP_WAKEUP_GPIO` (`0x05`) — not an EXT value. This works reliably in
Arduino; it does *not* in MicroPython (micropython#16839, #11787), which is why
the port away from the MicroPython prototype was worth making.

Deep sleep resets RAM, so state lives in RTC slow memory:

```cpp
struct PersistState {
  uint32_t magic;          // "WCTR" - validity marker across sleeps
  int32_t  latE7, lonE7;   // degrees x 1e7, keeps float out of slow mem
  uint8_t  sats;
  uint32_t fixTicks;       // millis() when acquired
  uint32_t nextSendTicks;
  uint8_t  moving, settled, hadFix;
};
```

A parked chair re-reports its last position for free instead of buying another
13.9 J acquisition.

`Serial.flush()` runs before `esp_deep_sleep_start()`. Deep sleep does not
flush the CDC, and since a wake re-enters `setup()`, unflushed logs make a
running board look dead.

---

## 6. Firmware quirks of this modem

Verified against a live SIM7600CE-T, firmware `LE20B05SIM7600M22`
(`AT+CSUB: MDM9x07_LE20_S_22_V1.14_240221`). These are **not** what the
datasheet says:

| Behaviour | Detail |
|---|---|
| **No AT data channels** | `AT+CIPSOCK`, `AT+CIPSTART`, `AT+CIPSTATUS`, `AT+CIFSR`, all `AT+CHTTP*`, all `AT+CMQ*` return `ERROR`. SMS and `AT+CDNSGIP` work. **No MQTT over cellular on this module.** |
| Misleading socket state | `AT+CIPACT?`, `AT+CIPOPEN?`, `AT+CIPCLOSE?` all respond correctly — the machinery is there, the open/connect verbs are not. |
| **GNSS is `AT+CGPS`** | `AT+CGNSPWR`, `AT+CGNSINF`, `AT+GPSINFO`, `AT+CGPSPORT` all return `ERROR`. Generic SIM7600 GNSS examples will not work. |
| `AT+CGPSNMEAPORTCFG=3` is a trap | Routes NMEA onto the AT port and wedges the command channel. The driver **rejects** port 3; use port 1. |
| Bare identity replies | With `ATE0`, `AT+CGMM` → `SIM7600CE-T`, no `+CGMM:` prefix, with leading blank lines. |
| URC prefix traps | `+CPIN`, `+CEREG`, `+CREG`, `+CGREG`, `+CMSS`, `+CDEL` share a prefix with query responses and must **not** be filtered as unsolicited, or the reply to the query is discarded. |
| `AT+CIFSR` broken | Use `AT+CGPADDR`. |
| `AT+COPS=0,2` ignored | Accepted, then ignored. The name is text in the default format anyway; only the technology is numeric, decoded internally. |
| GNSS settle time | `AT+CGPS=0` takes ~4 s to settle, emitting a bare `+CGPS: 0` on the way. `gnssOff()` waits; a premature status read reports the stale state. |
| `sync()` cost | `attempts × (retries + 1) × 1 s`, because `command()` retries a timeout internally. `sync(30,500)` blocked ~90 s before reporting. |

Because of the first row, **TinyGSM and the community SIM7600 libraries will not
work here** — their network paths depend on `AT+CIPSOCK`/`AT+CIPSTART`.
`Sim7600At` exists because it was verified against what this module actually
does.

---

## 7. Input validation

AT is a line protocol, so an unvalidated argument containing CR, LF or a quote
would let a caller append arbitrary commands. Phone numbers, APNs and hostnames
are **whitelisted, not escaped**:

```cpp
modem.smsSend("+62\r\nAT+CGSN", "x", ref);  // false, and nothing is transmitted
modem.smsSend("+1", "x", ref);              // false: 7-15 digits required
modem.smsSend(num, body200chars, ref);      // false: one GSM-7 segment
```

SMS bodies reject control characters and anything over 160 GSM-7 characters
(multipart is unimplemented rather than silently wrong).

---

## 8. Files

| File | Purpose |
|---|---|
| `wheelchair_tracker.ino` | state machine, deep sleep, GNSS + SMS schedule |
| `Sim7600At.h` / `.cpp` | AT transport — ported from the verified MicroPython driver |
| `lis2dh.h` / `.cpp` | I²C accelerometer, low-power mode + threshold interrupt |
| `Nmea.h` / `.cpp` | incremental NMEA parser with checksum validation |
| `../test/run.sh` | host suite, 53 assertions |
| `../test/shim/` | minimal Arduino shims so the real `.cpp` files compile off-board |
| `../poc/` | MicroPython prototype, kept as the verified reference |
| `../hardware.md` | bench checklist |

---

## 9. Status and open items

**Verified on hardware:** build, flash, boot, PWRKEY pulse, modem-sync timeout
path.

**Not yet verified:** everything past `setup()` — modem bring-up, GNSS capture
over UART1, LIS2DH detection and threshold tuning, deep-sleep wake.

Open:

- Modem hard power switching unimplemented (~10 mA idle dominates once the LED
  is gone).
- Send intervals are guesses pending a real tariff.
- GNSS fails indoors; `acquireFix()` blocks up to 45 s with the modem awake.
- Motion thresholds untuned — depend on sensor mounting relative to the frame.
- C3 Super Mini boards differ between sellers; confirm the silkscreen.

### Bench checklist

Run in order; each step gates the next.

1. **Desolder the LED.** Measure current before and after.
2. **Modem brings up.** `setup()` should print the model line and
   `sms=1 dns=1 gnss=1`.
3. **I²C scan.** `0x19` must respond; `WHO_AM_I` = `0x32`.
4. **LIS2DH registers.** Confirm `OUT_X_L` changes when you tilt the part.
5. **Threshold tuning.** Start `0x08` (0.5 g). Below ~`0x04`, floor vibration
   false-triggers — test on the real chair, not a desk.
6. **GNSS capture.** NMEA on UART1, needs outdoor sky view. Indoors will not
   succeed and that is expected.
7. **SMS round trip** before trusting anything on battery.
8. **Sleep current.** Above ~50 µA means something else is awake — check for a
   floating pin or the modem not sleeping.
9. **Endurance.** Log a full day parked, then a full day moving, and measure
   actual cell drain against the modelled budget.

---

## 10. Privacy

A device that continuously reports where a person is, over a public cellular
network, is a tracking device whether or not it is used that way. For a
disability aid the person should know it exists, hold the SIM, and be able to
switch it off physically — not merely via a setting in this firmware.

Set `OWNER` in the sketch to the number that should receive the pings, and
confirm that the person carrying the device agrees to it.