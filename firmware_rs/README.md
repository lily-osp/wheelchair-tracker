# firmware_rs — the firmware

Rust (`no_std`, esp-hal): motion-gated location tracker on ESP32-C3 SuperMini +
SIM7600CE-T + LIS2DH.

**Status: partially bench-verified on hardware.** All
parsing/classification/validation logic is host-tested (40 tests, `cargo test`
command below). Board smoke (2026-10-29, no peripherals): flash + boot ✔,
esp-hal/esp-rtos init ✔, RTC-fast state surviving deep sleep ✔, exact
`mono_ms` restoration on timer wake ✔, stable wake → classify → report → sleep
loop ✔, bounded absent-modem probe ✔.

**Bench-verified 2026-11-05 (ESP32-C3 ↔ SIM7600CE-T, SIM+antennas, Telkomsel
LTE):** AT link, SIM ready, registration, GNSS fix, and **end-to-end HTTP POST
to Adafruit IO (200)** via `AT+HTTP` (`http_probe`/`net_probe`; `http-probe`
landed in the feed). `PPP` (`ATD*99***1#`) was exercised but is **not usable**
on this carrier/firmware: TCP connects, the first request is reset with zero
bytes returned — so the uplink uses the module's own HTTP stack.

**Still unverified:** LIS2DH I2C/INT1 wake, `AT+CPOF` recovery, deep-sleep
current, and a live report from the full `firmware_rs` image carrying GNSS
coordinates (USB-JTAG serial re-enumerated after the full-image flash on this
bench; replug/monitor to capture). Treat sensor paths as bench-pending until
`docs/hardware.md`'s checklist runs.

## Issues #1–#6 fixed by construction

The tracker repo's open issues #1–#6 share one root cause: the original
prototype assumed `loop()` runs forever, but every cycle ends in deep sleep.
This firmware fixes them by construction instead of patching:

| Issue | Fix |
|---|---|
| #1 state lost on sleep | `#[esp_hal::ram(unstable(rtc_fast, persistent))]` static, magic + CRC16 (`state.rs`) |
| #2 hysteresis in function-local statics | streaks live in the persisted state (`motion.rs`) |
| #3 `millis()` resets on wake | firmware-accumulated `mono_ms` counter in RTC memory |
| #4 modem PWRKEY on every loop | `AT` probe first, PWRKEY pulse only if silent (`ensure_modem_on`) |
| #5 PWRKEY driven direct | N-MOSFET gate, idle LOW, HIGH pulse 1500 ms, level set at construction |
| #6 second UART for NMEA | single AT port, `AT+CGNSSINFO` with `AT+CGPSINFO` fallback |

#7–#10 (docs/test claims) are addressed by the honest satellite-count handling
in `gpsinfo.rs` (0 or omitted, never guessed) and the host test suite.

Uplink: **GPS + internet both via the SIM7600**. The ESP32's own WiFi/BT radio
is unused. Default channel is HTTP via the module's `AT+HTTP` stack to Adafruit
IO (verified 2026-11-05). SMS is the fallback (needs `owner` in
`adafruit.toml`). TLS is not enabled yet; the AIO key travels in the URL query
string over plain HTTP (see Known limitations).

## Stack

- `esp-hal` 1.2.2 (`unstable`) — drivers, deep sleep, `#[ram]` persistent statics
- `esp-rtos` 0.4 (embassy) — async runtime
- Modem-side `AT+HTTP` for the uplink (no TCP/IP stack on the ESP32). PPP was
  evaluated on hardware and abandoned: `ATD*99***1#` connects but the first
  request is reset with zero bytes returned on this carrier/firmware.
- no allocator, no TLS — plain HTTP POST, `heapless` strings only

## Building

Toolchain: stable, target `riscv32imc-unknown-none-elf` (pinned in
`rust-toolchain.toml`; `.cargo/config.toml` sets it as the default build
target).

```sh
nix develop -c bash -c 'cd firmware_rs && cargo build'   # or: cd firmware_rs && cargo build
cargo test --no-default-features --target x86_64-unknown-linux-gnu   # host tests
```

⚠️ The host test command **must** pass the explicit host target: esp-generate's
`.cargo/config.toml` makes the riscv target the default, and `cargo test`
without `--target` would try to link `std` into `no_std`.

### Flashing

With Nix (recommended — toolchain and `espflash` pinned by
[`../flake.nix`](../flake.nix), from the repo root):

```sh
nix run .#flash     # builds, flashes /dev/ttyACM0, attaches the monitor
nix run .#monitor   # monitor only
```

Bench probes (flash with espflash, capture serial separately — `espflash
--monitor` needs an interactive TTY):

```sh
cd firmware_rs
cargo build --bin modem_probe   # raw AT/UART link, no SIM/antenna needed
cargo build --bin http_probe    # module-side AT+HTTP POST to Adafruit IO
cargo build --bin net_probe     # same path through the async driver + uplink.rs
espflash flash --chip esp32c3 target/riscv32imc-unknown-none-elf/debug/<bin>
```

Without Nix, install `espflash` and from `firmware_rs/`:

```sh
cargo build
espflash flash --chip esp32c3 --monitor target/riscv32imc-unknown-none-elf/debug/firmware_rs
```

The C3 SuperMini programs over its native USB Serial/JTAG (`/dev/ttyACM0`); no
BOOT button is needed as long as the current app doesn't wedge USB. On Linux
you must be in the `dialout` group (udev grants `rw` to it).

Verified flash (2026-10-29): ESP32-C3 rev v0.4, 4MB, boot + first-cycle run OK.

### Configuration

- Board pins, cadences and thresholds live in [`src/config.rs`](src/config.rs).
- Adafruit IO + SIM settings live in [`adafruit.toml`](adafruit.toml) (copy from
  `adafruit.toml.example`): `key`, `user`, `host`, `port`, feed names, optional
  `owner` (SMS), and `apn` (carrier PDP context, e.g. `internet` for Telkomsel).
  The build script turns them into compile-time constants; edit the TOML, not
  the source. `adafruit.toml` is gitignored — never commit a real key.

## Pin map (must match [`../docs/hardware.md`](../docs/hardware.md))

| GPIO | Function |
|---|---|
| 4 | C3 TX → modem RXD |
| 5 | C3 RX ← modem TXD |
| 6 | N-MOSFET gate → modem PWRKEY (idle LOW) |
| 8 | I2C SDA (onboard LED removed first) |
| 9 | I2C SCL |
| 0 | LIS2DH INT1 (RTC wake pin; try GPIO3 if wake never fires) |

Modem I2C address: 0x19 (LIS2DH, SDO/SA0 high).

## Runtime model

- One task, no spawned futures that outlive a wake. A report wake borrows the
  modem UART, runs `AT+HTTP` (init → set URL/body → `HTTPACTION` → term), and
  returns before sleeping. No PPP session, no DNS-in-AT-context handoff.
- Wake causes (`wakeup_cause()`): GPIO = motion (IMU INT1), Timer = heartbeat.
  Motion wakes never touch the modem — they only advance hysteresis and, on a
  transition, pull the next report forward.
- Timer wakes restore `mono_ms` exactly from the armed `sleep_target_ms`;
  motion wakes bill one `POLL_MS` (sleep length is unknowable on that path).
- Cadences advance only on a successful send; failures retry after 60 s.
  Deep sleep always clamps to ≥ 2 s (the RTC deadline panics if too near) and
  ≤ 1 h; that upper clamp is the heartbeat backstop when the IMU is silent.
- Satellite count is reported only when a fix actually carried it — 0 or
  omitted otherwise (issue #7).
- Modem power-on is idempotent (probe before pulse); power-off prefers
  `AT+CPOF`, with a PWRKEY pulse as fallback.

## Bench checklist (before trusting any of it)

1. **UART + modem link**: `modem_probe` — `AT`/`ATI`/`AT+CGMR` answer, `+CPIN`
   READY with a SIM, `+CSQ`/`+CREG`/`+CPSI` show registration. ✔ 2026-11-05.
2. **AT+HTTP uplink**: `http_probe` or `net_probe` — `+HTTPACTION: 1,200,...`
   and the point lands in the Adafruit feed. ✔ 2026-11-05. Note: `AT+HTTPINIT`
   errors if a prior session is open (term + retry), the param is `CONTENT`
   (not `CONTENTTYPE`) on this firmware, and `AT+HTTPREAD` after POST returns
   ERROR — status comes from `+HTTPACTION`.
3. **PPP (diagnostic only)**: `dial_probe`/`net_probe`'s PPP history confirmed
   the carrier resets port-80 traffic on this path; do not use PPP for the
   uplink.
4. **INT1 wake**: verify the IMU actually wakes the chip (GPIO0 path). If not,
   rewire to GPIO3 and change `IMU_INT1`. The pull-down on the wake pin is
   load-bearing — a floating pad wakes immediately and forever.
5. **`AT+CPOF`**: confirm the module powers down and that the next `AT` probe
   (not PWRKEY) is the right recovery after it.
6. **RTC memory**: verify `mono_ms` advances across sleeps by comparing two
   consecutive report timestamps.
7. **Full image report**: flash `firmware_rs`, capture serial, confirm a live
   `http=200` report with real GNSS coordinates lands in the feed.
8. **SMS body length** (if using SMS fallback): keep reports under 160 chars.

## Known limitations

- **No TLS.** Plain HTTP: the Adafruit key is in the URL query string and the
  body is in the clear (operators encrypt at the link layer only). Enabling TLS
  means probing `AT+HTTPSSL=1` + `AT+CSSLCFG` on this firmware (issue #16) or
  adding a TLS-capable path — not done yet.
- **PPP unusable on this path.** `ATD*99***1#` negotiates but port-80 requests
  were reset with no response (2026-11-05, Telkomsel). The module-side `AT+HTTP`
  stack works reliably; keep PPP probes as diagnostics only.
- **No WiFi/BT.** Intentional — the modem is the radio.
