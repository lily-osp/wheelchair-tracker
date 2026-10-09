"""
LIS2DH / LIS2DH12 accelerometer driver - I2C, MicroPython.

Purpose here is motion gating, not measurement: the chair's state machine needs
to know "is this moving?" so it can decide whether to pay for a GNSS
acquisition. The device's low-power mode plus a threshold interrupt is exactly
the right tool.

Register map per LIS2DH12 datasheet. I2C address is 0x19 with SDO/SA0 high
(the usual breakout wiring), 0x18 with it pulled low.

The module stays in low-power mode throughout (~3 uA) and is never placed in
high-performance mode - there is no reason to for a yes/no motion question.
"""

from micropython import const

WHO_AM_I = const(0x0F)          # -> 0x32 for LIS2DH family
CTRL_REG1 = const(0x10)
CTRL_REG2 = const(0x11)
CTRL_REG4 = const(0x1F)
CTRL_REG5 = const(0x20)
STATUS_REG = const(0x1E)
STATUS_REG_AUX = const(0x7F)
INT1_CFG = const(0x09)
INT1_SRC = const(0x0D)
INT1_THS = const(0x07)
INT1_DUR = const(0x08)
OUT_X_L = const(0x28)

# CTRL_REG1: low-power mode, 10 Hz ODR, all axes on.
#   ODR[3:0] = 0010 -> 10 Hz (lowest rate still useful for gating)
#   LPen     = 1    -> low-power mode, ~3 uA
#   BDU      = 1    -> block data update, avoids reading mid-conversion
#   XYZ_EN   = 111
CTRL_REG1_LP_10HZ = const(0x2A)

# CTRL_REG4: ±2 g, so a wheelchair bump is well inside range without clipping.
CTRL_REG4_2G = const(0x08)

# INT1_CFG: interrupt on any of the three axes.
INT1_CFG_ANY_AXIS = const(0xFD)


class LIS2DH:
    def __init__(self, i2c, addr=0x19, int1_pin=None):
        self.i2c = i2c
        self.addr = addr
        self._int1 = int1_pin

    def _reg(self, reg, data):
        self.i2c.writeto(self.addr, bytes([reg] + data))

    def _read(self, reg, n):
        return self.i2c.readfrom(self.addr, n, reg)

    def probe(self):
        """True if the WHO_AM_I register matches a known LIS2DH value."""
        try:
            who = self._read(WHO_AM_I, 1)[0]
        except OSError:
            return False
        # LIS2DH12 reports 0x32; some LIS3DH-family clones report 0x33/0x2A.
        return who in (0x32, 0x33, 0x2A)

    def init(self, odr_10hz=True, threshold=0x10, duration=0):
        """
        Low-power mode at 10 Hz with a threshold interrupt on INT1.

        `threshold` is a raw 8-bit magnitude in 1/16 g, so 0x10 = 16/16 = 1 g.
        1 g is far too twitchy for a chair on a real floor - start around 0x08
        (0.5 g) and tune against actual use. Below ~0x04 you will get constant
        false wakes from vibration.
        `duration` is the number of consecutive samples above threshold before
        the interrupt fires. 0 means fire on the first one.
        """
        if not self.probe():
            raise OSError("no LIS2DH at I2C 0x%02X (check SDO/SA0 and wiring)" % self.addr)

        self._reg(CTRL_REG1, [CTRL_REG1_LP_10HZ if odr_10hz else 0x2A])
        self._reg(CTRL_REG4, [CTRL_REG4_2G])
        self._reg(CTRL_REG2, [0x00])           # no high-pass filter
        self._reg(INT1_CFG, [INT1_CFG_ANY_AXIS])
        self._reg(INT1_THS, [threshold])
        self._reg(INT1_DUR, [duration])
        self._reg(CTRL_REG5, [0x00])           # INT1 routed to INT1 pad
        return self

    def _status1(self):
        return self._read(STATUS_REG, 1)[0]

    def data_ready(self):
        return bool(self._status1() & 0x08)

    def accel(self):
        """
        (ax, ay, az) in mg. Blocking until the data-ready bit sets, bounded by
        `timeout_ms` so a dead sensor cannot hang the tracker.
        """
        deadline = self._ticks() + 200
        while not self.data_ready():
            if self._ticks() > deadline:
                return None
        raw = self._read(OUT_X_L, 6)
        # 12-bit left-justified within each 16-bit word, scale is 1 mg/LSB in
        # the ±2 g range.
        out = []
        for i in (0, 2, 4):
            v = (raw[i] | (raw[i + 1] << 8)) >> 4
            if v > 2047:
                v -= 4096
            out.append(v)
        return tuple(out)

    def motion_int(self):
        """
        True if the accelerometer's own threshold interrupt has fired.

        Reads and clears INT1_SRC (the read side-effect is how ST clears it).
        This is the cheap path: no conversion, no register walking, one byte.
        """
        return bool(self._read(INT1_SRC, 1)[0] & 0x01)

    def clear_int(self):
        self._read(INT1_SRC, 1)

    def magnitude(self):
        """Vector magnitude in mg. ~1000 when stationary, any orientation."""
        a = self.accel()
        if a is None:
            return None
        return int((a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) ** 0.5)

    def is_moving(self, idle_threshold=200, samples=3, timeout_ms=300):
        """
        Polling fallback for when the C3 has just woken and the interrupt latch
        is not trustworthy. Returns True if the acceleration magnitude strayed
        from 1 g by more than `idle_threshold` mg on `samples` consecutive reads.

        Magnitude rather than a per-axis deviation, because a parked chair is
        still tilted in whatever way its frame sits - only the vector length is
        orientation-independent, and that is the quantity gravity fixes at 1 g.

        Used as a cross-check in the tracker's main loop so a missed interrupt
        does not silently read as "parked".
        """
        hits = 0
        deadline = self._ticks() + timeout_ms
        while hits < samples and self._ticks() < deadline:
            mag = self.magnitude()
            if mag is None:
                break
            if abs(mag - 1000) > idle_threshold:
                hits += 1
            else:
                hits = 0
        return hits >= samples

    @staticmethod
    def _ticks():
        from time import ticks_ms
        return ticks_ms()