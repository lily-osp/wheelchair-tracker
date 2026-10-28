# firmware_rs — Rust port

Pure-Rust (`no_std`) port of the [`wheelchair_tracker/`](../wheelchair_tracker/)
Arduino sketch: motion-gated location tracker on ESP32-C3 SuperMini +
SIM7600CE-T + LIS2DH.

**Status: UNTESTED ON HARDWARE.** All parsing/classification/validation logic is
host-tested (38 tests, `cargo test` command below); the transport glue (AT
timings, PPP dial, sleep/wake path) is written against the verified C++
prototype's quirks but has never run on the board. Treat a first flash as a
bench exercise, not a deployment.

## Why a second firmware

The open issues #1–#6 of the tracker repo share one root cause: the sketch
assumes `loop()` runs forever, but every cycle ends in deep sleep. This port
fixes them by construction instead of patching:

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
is unused. Default channel is SMS (verified path); HTTP-over-PPP is opt-in
(`config::REPORT_VIA`).

## Stack

- `esp-hal` 1.2.2 (`unstable`) — drivers, deep sleep, `#[ram]` persistent statics
- `esp-rtos` 0.4 (embassy) — async runtime
- `embassy-net` 0.9 + `embassy-net-ppp` 0.3 (`ppproto` 0.2) — PPPoS over the
  modem's AT UART (`ATD*99***1#`), because the module's AT data channels
  (`AT+CIPSTART`, CIPSOCK) all return ERROR on firmware
  LE20B05SIM7600M22
- no allocator, no TLS — plain HTTP POST, `heapless` strings only

## Building

Toolchain: stable, target `riscv32imc-unknown-none-elf` (pinned in
`rust-toolchain.toml`; `.cargo/config.toml` sets it as the default build
target).

```sh
cargo build                      # firmware (default feature: firmware)
cargo test --no-default-features --target x86_64-unknown-linux-gnu   # host tests
```

⚠️ The host test command **must** pass the explicit host target: esp-generate's
`.cargo/config.toml` makes the riscv target the default, and `cargo test`
without `--target` would try to link `std` into `no_std`.

### Flashing

`espflash` is not installed. Either:

```sh
cargo install espflash
espflash flash --port /dev/ttyUSB0 target/riscv32imc-unknown-none-elf/debug/firmware_rs
```

or use `esptool.py` with the standard C3 offsets:

```
0x0    bootloader.bin   (build artifacts: bootloader/…)
0x8000 partition-table.bin
0xe000 boot_app0.bin
0x10000 firmware_rs.bin
```

### Configuration

Everything lives in [`src/config.rs`](src/config.rs) — set `OWNER` and
`SERVER_HOST` before flashing; both are placeholders.

## Pin map (must match `hardware.md`)

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
  modem UART, dials PPP, POSTs, and drops the session before sleeping.
- Wake causes (`wakeup_cause()`): GPIO = motion (IMU INT1), Timer = heartbeat.
  Motion wakes never touch the modem — they only advance hysteresis and, on a
  transition, pull the next report forward.
- Timer wakes restore `mono_ms` exactly from the armed `sleep_target_ms`;
  motion wakes bill one `POLL_MS` (sleep length is unknowable on that path).
- Cadences advance only on a successful send; failures retry after 60 s.
  Deep sleep always clamps to ≥ 2 s (the RTC deadline panics if too near) and
  ≤ 1 h watchdog.
- Satellite count is reported only when a fix actually carried it — 0 or
  omitted otherwise (issue #7).
- Modem power-on is idempotent (probe before pulse); power-off prefers
  `AT+CPOF`, with a PWRKEY pulse as fallback.

## Bench checklist (before trusting any of it)

1. **PPP dial**: `ATD*99***1#` → `CONNECT` → IPCP negotiation. Never exercised
   on this board; the CIP* commands being broken does not imply PPP is, but it
   is unproven. Serial-monitor `AT+CGDCONT` and APN setup first.
2. **INT1 wake**: verify the IMU actually wakes the chip (GPIO0 path). If not,
   rewire to GPIO3 and change `IMU_INT1`. The pull-down on the wake pin is
   load-bearing — a floating pad wakes immediately and forever.
3. **`AT+CPOF`**: confirm the module powers down and that the next `AT` probe
   (not PWRKEY) is the right recovery after it.
4. **Escape sequence**: `+++` needs 1 s of silence both sides (`PPP_ESCAPE_GUARD_MS`
   = 1100). Verify `ATH0` gets an `OK`/`NO CARRIER` after a POST.
5. **RTC memory**: verify `mono_ms` advances across sleeps by comparing two
   consecutive report timestamps.
6. **SMS body length**: keep reports under 160 chars (one SMS part); the
   fixed-point coordinates plus flags fit, but a longer `why` string will not.

## Known limitations

- **No TLS.** The HTTP body travels in the clear (GSM operators encrypt at the
  link layer only). Server-side: accept-and-drop garbage, rate-limit by IMSI if
  you need real auth. TLS needs an allocator + `esp-mbedtls`, which conflicts
  with the no-heap budget.
- **No DNS after dial.** `AT+CDNSGIP` resolves in AT context only; the HTTP
  path connects to the resolved IP with a `Host:` header.
- **No WiFi/BT.** Intentional — the modem is the radio.
