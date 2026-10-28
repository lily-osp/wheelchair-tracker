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

## On third-party libraries

TinyGSM and the community SIM7600 libraries assume a working `AT+CIPSOCK`/
`AT+CIPSTART`. On this module those commands fail, so their network paths are
dead on arrival here. `firmware_rs/` bypasses the AT data layer entirely: its
internet path is PPP over the AT UART (`ATD*99***1#`) with embassy-net, and its
AT transport was written against this module's actual verified behaviour.
