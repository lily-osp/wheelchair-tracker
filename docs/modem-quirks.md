# Modem quirks — SIM7600CE-T, firmware LE20B05SIM7600M22

Verified against the live module over USB. The parsers in
`firmware_rs/src/at.rs` encode all of these — if you write your own transport,
read this first:

- **The probed AT data commands all fail.** `AT+CIPSOCK`, `AT+CIPSTART`,
  `AT+CIPSTATUS`, `AT+CIFSR`, all `AT+CHTTP*`, all `AT+CMQ*` return `ERROR` on
  this module — but the probe was not exhaustive: SIM7600 also documents
  `AT+NETOPEN`/`AT+CIPOPEN` and `AT+CMQTT*` sets that we did not test. What is
  certain: SMS and `AT+CDNSGIP` work, and the PPP dial (`ATD*99***1#`) is a
  separate data path — `firmware_rs/` uses that for internet.
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

## Verified over USB (bench, 2026-10-31)

Port mapping on the composite device (`1e0e:9001`, six interfaces):

- `/dev/ttyUSB3` — primary AT + PPP data port. `AT+CGMM` → `SIMCOM_SIM7600CE-T`,
  APN `cmnet`, registered on Telkomsel. PPP dial `ATD*99***1#` → `CONNECT`,
  IPCP gives `10.0.68.103` (remote `10.64.64.64`, DNS `192.168.136.30`);
  `ping -I ppp0 8.8.8.8` and `curl --interface ppp0` both succeed.
- `/dev/ttyUSB4` — secondary AT port (also answers `AT`).
- GNSS: `AT+CGPS=1` then poll `AT+CGNSSINFO` returns a live fix, e.g.
  `2,...,0613.04xx,S,10647.60xx,E,311026,1616xx,...` (≈ 6.13°S 106.48°E).
- Do **not** use `AT+CGPSNMEAPORTCFG=3` (wedges the AT channel); port 1 is safe.
- On firmware LE20B05SIM7600M22, `AT+CGNSSINFO` prefixes four status fields before
  lat/N/S/lon/E/W (`status,sats_used,sats_view,fix_mode,`); parse by locating the
  hemisphere markers, not by fixed index. `AT+CGPS=1` returns `ERROR` if GNSS is
  already on (idempotency must probe `AT+CGPS?` first).
- `AT+CGMR` is prefixed (`+CGMR: LE20B05SIM7600M22`); `AT+CGMI`/`AT+CGMM` are bare.

## Adafruit IO sink (configured 2026-10-31)

- Host `io.adafruit.com:80` (plain HTTP; no TLS in firmware). Key header
  `X-AIO-Key: <see adafruit.toml>`.
- Feeds: `location` (value `"lat,lon"`), `time` (value = GNSS unix seconds).
- Paths: `/api/v2/azzar_4/feeds/location/data`, `/api/v2/azzar_4/feeds/time/data`.
- Verified end-to-end over PPP: POST returns 200 and data appears in the feed.

Use `ttyUSB3` for any bench AT/PPP work; the CP2102 bridge on `/dev/ttyUSB0`
is separate and not needed for the modem.

## On third-party libraries

TinyGSM and the community SIM7600 libraries assume a working `AT+CIPSOCK`/
`AT+CIPSTART`. On this module those commands fail, so their network paths are
dead on arrival here. `firmware_rs/` bypasses the AT data layer entirely: its
internet path is PPP over the AT UART (`ATD*99***1#`) with embassy-net, and its
AT transport was written against this module's actual verified behaviour.
