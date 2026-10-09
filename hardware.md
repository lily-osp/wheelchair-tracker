# wheelchair-tracker hardware

## Pin map

Verify against the silkscreen on your board — C3 SuperMini pinouts differ
between sellers. GPIO18/19 are the USB Serial/JTAG pair and are unusable if you
program over USB, which is the only programming path on this board.

| C3 | To | Note |
|---|---|---|
| GPIO4 | modem RXD | UART TX |
| GPIO5 | modem TXD | UART RX |
| GPIO6 | PWRKEY gate | N-MOSFET gate, **not** PWRKEY direct |
| GPIO8 | LIS2DH SDA | onboard LED — remove it (see below) |
| GPIO9 | LIS2DH SCL | |
| 3V3 | LIS2DH VCC | fine, draws µA |
| 5V/VBUS | modem VBAT via boost | **not** 3V3 |
| GND | everything | mandatory common ground |

PWRKEY idles high on a pull-up to VBAT and the module powers on when pulled low
~1.5 s. Drive it through a 2N7002/AO3400; the C3 must only touch the gate.

The modem cannot be powered from the C3's 3.3 V rail. VBAT needs 3.4–4.3 V and
the radio pulls close to 2 A in transmit bursts. On 3V3 it browns out mid-burst
and drops off the network, and it will fail USB enumeration outright.

## Remove the onboard LED — do this first

The C3 SuperMini LED on GPIO8 draws 1.3–2 mA. Continuous:

```
2 mA × 3.3 V × 24 h = 158 J/day
```

Deep sleep for the same day at 5 µA is about 0.4 J. The LED is roughly **400×
the entire sleep budget** — it will dominate battery life no matter how good the
motion gating is.

Options, best first:

1. **Desolder the LED.** Cleanest, and frees GPIO8 for I²C.
2. **Remove the series resistor**, then drive GPIO8 low so the LED stays dark.
   GPIO8 then reads low, which is safe for I²C only if it is not driving SDA —
   so pick a different pin for SDA instead.
3. Drive GPIO8 high permanently. Saves nothing if the LED is between the pin and
   a rail that lights it on high — on this board the LED is active-low, so
   driving high does turn it off. This costs nothing and frees nothing.

Verify with a multimeter in series before and after. Do not trust the silkscreen
for which polarity is which.

## Power budget

| Item | Current | Notes |
|---|---|---|
| C3 deep sleep | ~5 µA | the design target |
| C3 active | ~30 mA | during a GNSS read cycle |
| LIS2DH low-power | ~3 µA | 10 Hz ODR, threshold interrupt armed |
| SIM7600 idle | ~10 mA | registered, PSM off |
| SIM7600 GNSS acquisition | ~120 mA | **~35 s** cold start |
| SIM7600 TX burst | up to 2 A | milliseconds, but must not brown out |

One GNSS fix:

```
35 s × 120 mA × 3.3 V ≈ 13.9 J
```

A full day of deep sleep is about 0.4 J. **One cold fix costs about 35 days of
sleep.** This single ratio is why `wheelchair_tracker.py` never acquires while
parked, and why it reports a stale cached fix instead of re-fixing.

Waking every 5 s to sample the accelerometer costs about 1.2 J/day — worth it,
and small next to a fix.

## Battery

Li-ion cell + boost to 4.0 V, sized for the two supplies separately:

- **C3 + LIS2DH** run from the cell's 3.7 V directly.
- **Modem** needs a boost converter that holds 4.0 V under a 2 A burst. A
  marginal one is the single most common cause of "it works on the bench but not
  in the chair" — brownouts look like flaky registration, not like power faults.

The modem must be switched **hard** to save power, or its ~10 mA idle will
outweigh everything else combined. This is not implemented yet — see the open
items below.

## GNSS antenna

Required, not optional. Without one there is no fix at all, not degraded
accuracy. Must be mounted clear of the chair's frame and any body — a metal
wheel or frame will attenuate badly. Keep it away from the boost converter's
inductor and the Li-ion cell.

## Bench checklist

Run in this order; each step gates the next.

1. **I2C scan.** `i2c.scan()` must list `0x19` (or `0x18` if SDO/SA0 is low).
   Nothing works until this passes.
2. **LIS2DH register readback.** Confirm `WHO_AM_I` returns `0x32` and that
   `OUT_X_L` changes when you tilt the device. Static readings mean the driver
   or the address is wrong, not the sensor.
3. **Motion threshold tuning.** Set `threshold` in `init()`, then shake the
   device. `0x08` (0.5 g) is a starting point. Vibration from a real floor will
   false-trigger below roughly `0x04`; verify on the actual chair, not a desk.
4. **GNSS sentence capture.** Confirm NMEA appears on the modem's UART1 and that
   `nmea.feed()` yields `fix.valid`. Needs outdoor sky view. Indoors this will
   not succeed and that is expected.
5. **SMS round trip.** Verify a ping arrives before trusting any of the above
   on battery.
6. **Current draw.** Measure deep-sleep current with the LED removed. If it is
   above ~50 µA something else is awake — check for a floating pin or the modem
   not entering sleep.
7. **Endurance.** Log a full day parked, then a full day moving, and measure
   actual cell drain. Compare against the modelled budget before shipping.

## Open items

- **Modem hard power switching is not implemented.** At 10 mA idle the modem will
  dominate the budget. Cutting VBAT requires a load switch plus proper PWRKEY
  sequencing and re-registration time, so it is a separate piece of work.
- **Intervals are unvalidated against real costs.** 60 s while moving is a guess
  based on plausibility, not on Telkomsel's actual tariff. Check the per-SMS
  price and recalculate before committing.
- **No fix timeout path.** `acquire()` blocks up to 45 s. If GNSS routinely fails
  indoors that is a long stall per attempt, and the modem stays awake throughout.
- **Motion thresholds are untuned** and depend entirely on how the sensor is
  mounted relative to the frame.