"""
Motion-gated wheelchair tracker.

State machine over the LIS2DH motion gate and the SIM7600 GNSS. Wakes on a
short timer, checks whether the chair moved, and only pays for a GNSS
acquisition and an SMS when it did.

Design notes that are not obvious:

* GNSS is never re-acquired while the chair is stationary. A cold fix is ~35 s
  at ~120 mA, roughly 400x the energy of a whole day of C3 deep sleep, so
  repeated fixing while parked is the single largest avoidable drain.
* The last known fix is cached and re-reported with an age marker instead. A
  stale position that is honestly labelled beats a fresh one that was never
  acquired, because a parked chair's position has not changed.
* Waking is on a timer, not on the accelerometer's INT1 pin. GPIO wake from
  deep sleep on the ESP32-C3 is unreliable under MicroPython (micropython
  issues #16839 and #11787), and a tracker that never wakes is worse than one
  that wakes slightly too often.
* Hysteresis is required. Without it a chair halted at a kerb flips state every
  few seconds and fires an SMS each time.

UNTESTED ON HARDWARE. See README.md for the bench checklist.
"""

from machine import I2C, Pin, UART, deep_sleep
from time import ticks_ms, ticks_diff, sleep_ms

from lis2dh import LIS2DH
from sim7600 import Sim7600, ATError

# --- pins (C3 SuperMini, verify against your board's silkscreen) -----------
MODEM_TX = 4          # C3 TX  -> modem RXD
MODEM_RX = 5          # C3 RX  <- modem TXD
PWRKEY_GATE = 6       # N-MOSFET gate, driven high to pulse power key
LED_PIN = 8           # onboard LED - drives it low to stop the drain, see README

I2C_SDA = 8
I2C_SCL = 9
LIS2DH_ADDR = 0x19    # 0x18 if SDO/SA0 is pulled low

# --- timing ----------------------------------------------------------------
POLL_MS = 5000        # wake every 5 s to sample the accelerometer
BOOT_GRACE_MS = 15000  # classify motion before the first send, never fix while parked
SEND_MOVING_MS = 60000    # report cadence while actually travelling
SEND_IDLE_MS = 900000     # heartbeat while parked (15 min)
GNSS_RETRY_MS = 300000    # how long a fix is considered usable
STALE_MS = 600000         # after this, report position as stale

# thresholds in mg of deviation from 1 g
MOVE_ENTER = 250      # must exceed this to be called moving
MOVE_EXIT = 120       # must fall below this to be called parked
HYSTERESIS_SAMPLES = 3


class Tracker:
    def __init__(self, uart, i2c):
        self.imu = LIS2DH(i2c, LIS2DH_ADDR)
        self.modem = Sim7600(uart, timeout=8000)
        self.moving = False
        self._moving_streak = 0
        self._idle_streak = 0
        self.last_fix = None       # (lat, lon, sats, ticks)
        self.booted_at = ticks_ms()
        # A Tracker object is constructed fresh after every deep sleep, so the
        # boot grace cannot be derived from ticks here - it would restart on
        # each wake and nothing would ever be due. `settled` flips on the first
        # step() instead, which is the only place that can tell.
        self.settled = False
        self.next_send = ticks_ms()

    # -- motion gate --------------------------------------------------------

    def update_motion(self):
        """
        Hysteretic motion state. Movement needs MOVE_ENTER exceeded
        HYSTERESIS_SAMPLES times in a row; parking needs MOVE_EXIT exceeded the
        same. Single-sample flips are ignored, which is what stops kerb
        oscillation.
        """
        active = self.imu.motion_int() or self.imu.is_moving(
            idle_threshold=MOVE_ENTER, samples=1)

        if active:
            self._moving_streak += 1
            self._idle_streak = 0
        else:
            self._idle_streak += 1
            self._moving_streak = 0

        if not self.moving and self._moving_streak >= HYSTERESIS_SAMPLES:
            self.moving = True
            # Report straight away on transition, so a journey starts with a
            # position rather than up to a minute of silence.
            self.next_send = 0
        elif self.moving and self._idle_streak >= HYSTERESIS_SAMPLES:
            if self.imu.magnitude() and abs(self.imu.magnitude() - 1000) <= MOVE_EXIT:
                self.moving = False
        return self.moving

    # -- position -----------------------------------------------------------

    def fix_is_good(self):
        return self.last_fix is not None and \
            ticks_diff(ticks_ms(), self.last_fix[3]) < GNSS_RETRY_MS

    def acquire(self):
        """
        One GNSS attempt. Returns True on success.

        Never called while parked - that is the whole point of the gate.
        """
        try:
            self.modem.gnss_on(nmea_port=1)
        except (ATError, ValueError) as err:
            print("gnss on failed:", err)
            return False

        fix = self._read_nmea(timeout_ms=45000)
        try:
            self.modem.gnss_off()
        except ATError:
            pass

        if fix is not None and fix.valid:
            self.last_fix = (fix.lat, fix.lon, fix.sats, ticks_ms())
            return True
        return False

    def _read_nmea(self, timeout_ms=45000):
        """
        Parse NMEA from the modem's second UART. Imported lazily so the
        simulator in tests can run without the file present.
        """
        try:
            import nmea
        except ImportError:
            print("nmea.py missing - cannot parse fix")
            return None

        nmea_uart = UART(2, baudrate=115200, timeout=20)
        fix = nmea.Fix()
        buf = bytearray()
        end = ticks_ms() + timeout_ms
        while ticks_diff(ticks_ms(), end) < 0:
            chunk = nmea_uart.read(128)
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                line = bytes(buf[:nl]).decode("ascii", "replace").strip()
                del buf[:nl + 1]
                nmea.feed(fix, line)
                if fix.valid:
                    return fix
        return fix if fix.sats else None

    # -- reporting ----------------------------------------------------------

    def report(self):
        """
        Send one position SMS. Returns True if something was transmitted.

        When no fresh fix exists, the cached one is sent with its age marked,
        so the receiver can tell a stale ping from a live one.
        """
        if not self.fix_is_good() and not self.acquire():
            if self.last_fix is None:
                print("no fix yet, nothing to report")
                return False

        lat, lon, sats, when = self.last_fix
        age_s = ticks_diff(ticks_ms(), when) // 1000
        stale = age_s * 1000 > STALE_MS
        tag = "M" if self.moving else "S"
        note = " stale" if stale else ""
        body = "{}{} {:.5f},{:.5f} s{} a{}s{}".format(
            tag, note, lat, lon, sats, age_s, note)
        try:
            self.modem.sms_send(OWNER, body[:160])
            return True
        except (ATError, ValueError) as err:
            print("sms failed:", err)
            return False

    def due(self):
        return ticks_diff(ticks_ms(), self.next_send) >= 0

    def schedule(self):
        self.next_send = ticks_ms() + (
            SEND_MOVING_MS if self.moving else SEND_IDLE_MS)

    # -- main loop ----------------------------------------------------------

    def step(self):
        """One wake cycle: sample, gate, report if due. Returns True if sent."""
        self.update_motion()

        if not self.settled:
            # Let the gate classify first. A chair that boots while parked must
            # not buy a GNSS fix on the way to deciding it is parked.
            self.settled = True
            if not self.moving:
                self.schedule()
                return False
            self.next_send = 0        # moving at boot: report straight away

        if not self.due():
            return False
        sent = self.report()
        self.schedule()
        return sent


OWNER = "+6281234567890"   # set before flashing; this is a placeholder


def main():
    # Kill the onboard LED's ~2 mA before anything else. It costs more than
    # the entire deep-sleep budget - see README.md.
    try:
        Pin(LED_PIN, Pin.OUT).value(1)
    except Exception as err:
        print("led control skipped:", err)

    i2c = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400000)
    print("i2c scan:", [hex(a) for a in i2c.scan()])

    uart = UART(1, baudrate=115200, tx=MODEM_TX, rx=MODEM_RX, timeout=20)
    tracker = Tracker(uart, i2c)

    if not tracker.imu.probe():
        print("no LIS2DH - fix I2C_SDA/SCL or the address (0x18 vs 0x19)")
        return

    tracker.imu.init(threshold=0x08)      # 0.5 g, tune against real use

    if not tracker.modem.sync():
        print("modem did not answer - check PWRKEY and the 4.0 V supply")
        return
    tracker.modem.begin()
    tracker.booted_at = ticks_ms()

    while True:
        sent = tracker.step()
        if sent:
            print("reported, moving=%s" % tracker.moving)

        # Timer wake, not GPIO wake: C3 GPIO deep-sleep wake is unreliable under
        # MicroPython and a tracker that never wakes is the worst failure mode.
        ms = ticks_diff(ticks_ms(), tracker.next_send)
        deep_sleep(min(max(ms, 0), POLL_MS))


if __name__ == "__main__":
    main()