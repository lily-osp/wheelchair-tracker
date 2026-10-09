"""
NMEA reader for GNSS started with modem.gnss_on(nmea_port=1).

The module streams raw NMEA sentences to a second UART. Wiring is TX-to-TX
crossed the usual way: module TX -> C3 RX. Nothing from the AT parser is
needed; just decode sentences as they arrive.
"""

from machine import UART
from time import ticks_ms

UART_ID = 2                       # the spare UART, distinct from the AT port
NMEA_BAUD = 115200                # NMEA port follows AT+IPR on this build


class Fix(object):
    __slots__ = ("lat", "lon", "alt_m", "speed_kmh", "course_deg",
                 "sats", "fix", "pdop", "utc")

    def __init__(self):
        self.lat = self.lon = self.alt_m = None
        self.speed_kmh = self.course_deg = None
        self.sats = self.pdop = None
        self.fix = 0                  # 0 none, 1 GPS, 2 DGPS
        self.utc = None

    @property
    def valid(self):
        return self.fix > 0 and self.lat is not None and self.lon is not None

    def __repr__(self):
        if not self.valid:
            return "<Fix no lock sats=%s>" % self.sats
        return "<Fix %.5f %.5f +/-%sm sats=%s pdop=%s>" % (
            self.lat, self.lon, self.alt_m or 0, self.sats, self.pdop)


def _to_deg(raw, deg_digits):
    """
    NMEA ddmm.mmmm / dddmm.mmmm -> signed decimal degrees. The degree field is
    fixed width, so the split is positional - counting the digits before the
    decimal point is wrong, because '03114.10336' is 3 degrees, not 31.
    """
    return float(raw[:deg_digits]) + float(raw[deg_digits:]) / 60.0


def checksum_ok(sentence):
    """True if the sentence's trailing XOR checksum is valid."""
    body, _, tail = sentence[1:].partition("*")
    try:
        return abs(sum(ord(c) for c in body) & 0xFF) == int(tail[:2], 16)
    except ValueError:
        return False


def feed(fix, sentence):
    """Update `fix` from one NMEA sentence. Returns True if it applied."""
    sentence = sentence.strip()
    if not sentence.startswith("$") or "*" not in sentence or not checksum_ok(sentence):
        return False
    body = sentence[1:].partition("*")[0]
    parts = body.split(",")
    # Tag carries a 2-char talker id on multi-constellation receivers
    # ('GN' on this module), so match on the trailing sentence name only.
    tag, fields = parts[0][-3:], parts[1:]

    # Field maps, after the sentence tag. `?` means the field may be empty.
    #   GGA  0 utc, 1 lat, 2 NS, 3 lon, 4 EW, 5 quality, 6 sats, 7 hdop, 8 alt
    #   RMC  0 utc, 1 status, 2 lat, 3 NS, 4 lon, 5 EW, 6 kn, 7 course, 8 date
    if tag == "GGA" and len(fields) >= 9:
        fix.utc = fields[0] or fix.utc
        if fields[1] and fields[3]:
            fix.lat = _to_deg(fields[1], 2)
            if fields[2] == "S":
                fix.lat = -fix.lat
            fix.lon = _to_deg(fields[3], 3)
            if fields[4] == "W":
                fix.lon = -fix.lon
        if fields[5]:                       # 0 = invalid, 1 = GPS, 2 = DGPS
            fix.fix = int(fields[5])
        fix.sats = int(fields[6]) if fields[6] else fix.sats
        fix.pdop = float(fields[7]) if fields[7] else fix.pdop
        fix.alt_m = float(fields[8]) if fields[8] else fix.alt_m

    elif tag == "RMC" and len(fields) >= 9:
        if fields[1] == "A":                # status 'A' = valid fix
            if fields[2] and fields[4]:
                fix.lat = _to_deg(fields[2], 2)
                if fields[3] == "S":
                    fix.lat = -fix.lat
                fix.lon = _to_deg(fields[4], 3)
                if fields[5] == "W":
                    fix.lon = -fix.lon
            if fields[6]:
                fix.speed_kmh = float(fields[6]) * 1.852   # knots -> km/h
            if fields[7]:
                fix.course_deg = float(fields[7])
            if fields[0]:
                fix.utc = fields[0]
            fix.fix = 1

    elif tag == "GSA" and len(fields) >= 3:
        try:
            fix.sats = int(fields[2])
        except ValueError:
            pass
        if len(fields) > 15 and fields[15]:
            fix.pdop = float(fields[15])

    else:
        return False
    return True


def main():
    nmea = UART(UART_ID, baudrate=NMEA_BAUD, timeout=20)
    fix = Fix()
    buf = bytearray()
    last_report = 0
    print("listening for NMEA - press Ctrl-C to stop")
    while True:
        chunk = nmea.read(128)
        if not chunk:
            continue
        buf.extend(chunk)
        while True:
            nl = buf.find(b"\n")
            if nl < 0:
                break
            line = bytes(buf[:nl]).decode("ascii", "replace").strip()
            del buf[:nl + 1]
            if feed(fix, line):
                now = ticks_ms()
                if now - last_report > 2000:
                    last_report = now
                    print(fix)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("stopped")