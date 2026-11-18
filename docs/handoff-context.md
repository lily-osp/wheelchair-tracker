# Handoff / Replication Context (ESP32-C3 Wheelchair Tracker)

Snapshot for replicating and continuing work on another machine.  
Repo: `lily-osp/wheelchair-tracker`  
Branch: `main` @ `694b509` (origin/main synced at time of dump)  
Working tree: clean (this dump created as a new doc — commit if you want it tracked).

> **Secrets:** `firmware_rs/adafruit.toml` is **git-ignored** and contains the real Adafruit IO key/username/APN. It is **not** included here. Copy `firmware_rs/adafruit.toml.example` → `firmware_rs/adafruit.toml` and fill values (see below). Never commit `adafruit.toml`.

---

## 1) Hardware & wiring (bench-verified)

- **MCU:** ESP32-C3 SuperMini (rev v0.4, 4 MB, MAC `3c:0f:02:a3:ce:54` in this bench), USB-JTAG/serial `303a:1001`, appears as `/dev/ttyACM0`.
- **Modem:** SIM7600CE-T on Waveshare SIM7600X 4G/LTE Cat-1 HAT. Firmware seen: `LE20B05SIM7600M22` / `MDM9x07_LE20_S_22_V1.14_240221`, IMEI observed on bench unit.
  - UART: **C3 TX GPIO4 → modem RXD**, **C3 RX GPIO5 ← modem TXD**, 115200 8N1.
  - HAT UART jumper **B** (module ↔ host header). VCCIO **3.3 V**. PWR↔3V3 fitted for auto power-on (current bench: no PWRKEY MOSFET; GPIO6/PWRKEY gate left unconnected/floating is harmless in this setup).
  - Power: common GND; modem 5V/VBAT per HAT; C3 via USB. SIM + antennas required for network/GNSS.
- **IMU:** LIS2DH/LIS3DH-family clone (WHO_AM_I `0x33`) on I2C.
  - **SDA GPIO8, SCL GPIO9**, INT1 **GPIO0** (RTC-capable wake pin).
  - **I2C address `0x18`** (SA0/SDO low) — bench-verified 2026-11-05 (not `0x19`).
  - Onboard LED on GPIO8 should be removed (per hardware notes) to avoid bus issues.
- Pin/threshold/cadence source of truth: `firmware_rs/src/config.rs` and `docs/hardware.md`.

## 2) Software / environment

- **Firmware:** Rust `no_std`, `esp-hal` ~1.2.2 (unstable), `esp-rtos` 0.4 (embassy), `embassy-time` 0.5, `heapless`, `bytemuck`. No allocator, no TLS. Target `riscv32imc-unknown-none-elf` (see `firmware_rs/rust-toolchain.toml`).
- **Toolchain (local, known-good):** `rustc 1.99.1`, `cargo 1.99.1`. Flash/monitor: `espflash` (e.g. `nix run nixpkgs#espflash`), `espflash 4.7.0` used on bench.
- **Nix:** `flake.nix` provides pinned dev shell and apps (`#flash`, `#monitor`, `#probe`). Note: the pinned flake Rust may lag for some crate builds on this target; **local cargo + nixpkgs espflash** is a reliable fallback.

```sh
# build firmware + host tests
cd firmware_rs
cargo build --bins
cargo test --no-default-features --target x86_64-unknown-linux-gnu   # 40 passed

# flash (pick a bin/elf)
espflash flash --chip esp32c3 target/riscv32imc-unknown-none-elf/debug/<bin>

# serial capture (USB-JTAG; espflash --monitor may be non-interactive-unfriendly)
python3 /var/tmp/scratch/serial_read.py /dev/ttyACM0 120
```

A DTR/RTS-safe, reconnect-tolerant reader was used at `/var/tmp/scratch/serial_read.py` (retries open/read; useful across resets/deep-sleep USB dropouts). Recreate if missing.

## 3) Configuration

`firmware_rs/build.rs` generates compile-time AIO/APN constants from `firmware_rs/adafruit.toml` (git-ignored). Copy the example:

```toml
# firmware_rs/adafruit.toml  (DO NOT COMMIT)
[aio]
host = "io.adafruit.com"
port = 80
key = "YOUR_AIO_KEY"
user = "YOUR_AIO_USERNAME"
location_feed = "location"
time_feed = "time"
owner = "YOUR_OWNER_NUMBER"   # SMS fallback
apn = "internet"              # Telkomsel example; carrier PDP context
```

- Channel is chosen at runtime by the GPIO10 mode switch (`read_mode()` in
  `bin/main.rs`): open/not fitted = `ReportVia::Http`, closed to GND =
  `ReportVia::Sms`; `MODE_SWITCH_FITTED = false` falls back to
  `REPORT_VIA_DEFAULT` (Http). The selected channel is persisted in
  `PersistState::via` and also sets the cadence (`ReportVia::moving_ms()` /
  `idle_ms()`): 60 s / 15 min on HTTP, 5 min / 1 h on SMS. Bench with the
  `mode_probe` binary.
- HTTP path (switch open). HTTP path: modem-side `AT+HTTP` POST to `http://{host}/api/v2/{user}/feeds/{location_feed}/data?X-AIO-Key=...` with JSON body `{"value":"...","lat":<deg>,"lon":<deg>,"created_at":<unix>}` (value is a status/lat,lon string in firmware; map uses `lat/lon` fields).

## 4) Bench binaries (`firmware_rs/src/bin/`)

| Bin | Purpose | Notes |
|---|---|---|
| `firmware_rs` (`main.rs`) | Full tracker: motion-gated, RTC-persisted state, deep sleep, GNSS + HTTP report | Production path; USB-JTAG logs hard to observe across deep sleep |
| `modem_probe` | Raw AT/UART + identity/CPIN/CSQ/CREG/CPSI/CGPS poll | No SIM/antenna needed for AT link |
| `imu_probe` | LIS2DH dual-addr probe, reg dump, raw OUT/mag + INT1_SRC/GPIO0 level | Confirms I2C/accel; shake-test INT1 pin |
| `http_probe` | Blocking/module-side AT+HTTP POST (diagnostic) | Proved `+HTTPACTION: 1,200,...` |
| `net_probe` | Async `Sim7600` + `uplink::post_json` (production uplink path), stays awake | Reliable uplink validation |
| `report_probe` | Full flow without deep sleep: bring-up + GNSS fix + AT+HTTP POST (firmware-style payload) | Best for observable GNSS+HTTP integration |
| `sleep_wake_probe` | Arms LIS2DH INT1 (GPIO0) and enters deep sleep; on GPIO wake reboots and stays awake printing success | Validates deep-sleep interception |
| `dial_probe` | Raw `ATD*99***1#` diagnostic | PPP not used |

## 5) Key findings / quirks (important)

- **Uplink = AT+HTTP (not PPP).** On SIM7600CE-T / `LE20B05SIM7600M22` (Telkomsel), `ATD*99***1#` PPP negotiated but the first HTTP request was **reset with zero bytes**. Module-side `AT+HTTP` works reliably (`+HTTPACTION: 1,200,...`), bench-verified posting to Adafruit IO.
- **AT+HTTP specifics (this firmware):**
  - `AT+HTTPINIT` can ERROR if a session is left open → `AT+HTTPTERM` then retry.
  - Use `AT+HTTPPARA="CONTENT","application/json"` (not `"CONTENTTYPE"` — that errors here); without correct content Adafruit may 406. `ACCEPT` also set.
  - Custom headers not supported via HTTPPARA; **AIO key is in URL query string** (`?X-AIO-Key=...`) over **plain HTTP** (no TLS yet).
  - `AT+HTTPREAD` after POST may ERROR; success is taken from `+HTTPACTION` status.
  - `uplink.rs` builds URL/body, handles init/retry/term, waits for `DOWNLOAD` and `+HTTPACTION`.
- **APN** must be set for PDP context (e.g. Telkomsel `internet`) via `adafruit.toml` → `CFG_APN`.
- **LIS2DH:** address **0x18** on this board. **Bug fixed:** multi-byte OUT reads require auto-increment (`OUT_X_L | 0x80`) in `lis2dh.rs`; without it accel reads were all-zero. Host tests pass (40).
- **INT1 deep-sleep wake:** LIS2 `INT1_SRC` shows IA/activity and accel is good, but **GPIO0 level not observed high** in static samples and a 300s `sleep_wake_probe` capture saw no GPIO wake. Suspect INT1 pin routing/level/polarity or 6D-config behavior for a sustained wake level — verify with `imu_probe` during shake and confirm LIS2 **INT1** pad → **C3 GPIO0** (not INT2).
- **USB-JTAG observability:** device/tty can flap; USB-JTAG may drop during deep sleep, so full-image serial traces are hard — prefer `net_probe`/`report_probe` (awake) for uplink/GNSS validation.

## 6) Current bench status (last checked)

- Repo `main` pushed through `694b509`, clean working tree.
- ESP32-C3 enumerates as `/dev/ttyACM0` (when stable). USB can be flaky — replug/cable/port if missing.
- **LIS2DH:** ✔ connected/readable @ `0x18`, mag ~1 g. INT1→GPIO0 physical wake **not yet confirmed**.
- **SIM7600:** ✔ AT/UART link, module ID OK. **SIM not detected** at last check (`+CME ERROR: SIM not inserted`, `CREG 0,2`, `NO SERVICE`) — reseat SIM/antenna and re-run `modem_probe`.
- **Internet → Adafruit:** ✔ proven earlier (`http_probe`/`net_probe`, `POST 200`, points in `location` feed). A manual `lat/lon` point can be posted via REST for dashboard checks.
- **Full `firmware_rs` image:** flashed previously; live GNSS+HTTP report not serial-confirmed due to deep-sleep/USB observability (uplink path itself proven via probes). `report_probe` is the best observable end-to-end test.

## 7) Quick replication checklist

1. Install Rust target/toolchain per `firmware_rs/rust-toolchain.toml` (+ `espflash`) or `nix develop`.
2. Create `firmware_rs/adafruit.toml` from example (key/user/feeds/apn).
3. `cd firmware_rs && cargo build --bins && cargo test --no-default-features --target x86_64-unknown-linux-gnu`.
4. Wire per §1, connect `/dev/ttyACM0`.
5. Flash probes in order: `modem_probe` (AT/SIM), `imu_probe` (I2C/accel/INT1), `net_probe` or `report_probe` (cellular+GNSS→Adafruit 200).
6. For production behavior: flash `firmware_rs`; expect deep-sleep USB gaps, validate via feed.

## 8) Useful references in repo

- `README.md`, `firmware_rs/README.md` — architecture, build/flash, runtime model, limitations.
- `docs/hardware.md` — HAT jumpers, power, bench checklist.
- `docs/modem-quirks.md` — SIM7600 AT quirks.
- `flake.nix` — dev shell/apps.
- Recent commits: `02476fc` (AT+HTTP uplink, probes, drop PPP), `6591efe` (LIS2 auto-inc + addr 0x18 + imu_probe), `3081861`/`694b509` (imu/shake sampling, sleep_wake_probe).
