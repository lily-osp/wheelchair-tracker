"""
MicroPython driver for the SIMCom SIM7600CE-T (Waveshare 4G HAT) over AT/UART.

Written against a live unit: SIM7600CE-T, firmware LE20B05SIM7600M22
(AT+CSUB: MDM9x07_LE20_S_22_V1.14_240221), SIM7600CE-T 4G HAT.

Two things this firmware does that the datasheet does not:

  * GNSS is driven by AT+CGPS / AT+CGPSNMEAPORTCFG, NOT the usual AT+CGNSPWR.
    Every standard CGNSPWR/GPSINFO/CGNSINF form returns ERROR on this build.
  * AT-based data channels are absent: AT+CIPSOCK, AT+CIPSTART, AT+CIPSTATUS,
    AT+CIFSR, the whole AT+CHTTP* HTTP set and the whole AT+CMQ* MQTT set all
    return ERROR. SMS and AT+CDNSGIP do work. Call capabilities() at boot and
    branch on it rather than assuming.

Runs on MicroPython. Imports on CPython too, so the same file can be exercised
against a real module over a USB-serial adapter from a workstation.
"""

try:
    from time import sleep_ms, ticks_ms, ticks_diff
except ImportError:  # CPython, for host-side testing
    from time import sleep, time

    def sleep_ms(ms):
        sleep(ms / 1000.0)

    def ticks_ms():
        return int(time() * 1000)

    def ticks_diff(a, b):
        return a - b


__all__ = [
    "Sim7600", "Response", "ATError", "ATTimeout", "CMError", "CMSError",
    "NotSupportedError", "SimLockedError",
]

# Result codes that terminate a command response.
_TERMINAL_OK = "OK"
_TERMINAL_ERR = ("+CME ERROR", "+CMS ERROR", "NO CARRIER", "ABORTED", "BUSY")

# Lines starting with "+" that arrive unprompted. Anything else beginning with
# "+" is treated as payload and must be consumed by the pending read, or it is
# lost. SMS bodies can legitimately start with "+", so this list must stay
# narrower than "everything".
#
# Query responses that share a prefix with a URC (+CPIN, +CEREG, +CREG,
# +CGREG, +CMSS, +CDEL) are deliberately absent: treating them as URCs silently
# discards the reply to the query that asked for them.
_URC_PREFIXES = (
    "+CMTI", "+CMT", "+CUSD", "+CGEV", "+CIEV", "+CRING", "+RING", "+POLL",
    "+PBREADY", "+CTS", "+CERR",
)

# Default APN, correct for Telkomsel (SIM card reports MCC 510).
DEFAULT_APN = "cmnet"

# +COPS access-technology codes. Format 2 does not stick on this firmware -
# AT+COPS=0,2 is accepted and then ignored - so the code has to be decoded here.
TECH_NAMES = {
    "0": "GSM", "1": "GSM Compact", "2": "GSM", "3": "GSM w/EGPRS",
    "4": "UMTS", "5": "CDMA", "6": "CDMA2000 1xRTT", "7": "LTE", "8": "EC-GSM",
    "9": "HSDPA", "10": "HSUPA", "11": "HSPA", "12": "UMTS", "13": "E-UTRA",
    "14": "HSDPA", "15": "HSUPA", "16": "HSPA", "17": "E-UTRA", "18": "CDMA2000 1xEV-DO",
    "19": "HRPD", "20": "HSPA", "21": "HSUPA", "22": "HSPA", "23": "E-UTRA",
    "24": "HSDPA", "25": "HSUPA", "26": "HSPA", "27": "E-UTRA", "28": "CDMA2000 1xEV-DO",
}


# --------------------------------------------------------------------------
# Errors
# --------------------------------------------------------------------------

class ATError(Exception):
    """Module replied ERROR."""


class ATTimeout(ATError):
    """No terminating result code before the deadline."""


class CMError(ATError):
    """+CME ERROR - mobile termination error. `code` is the numeric reason."""

    def __init__(self, code, cmd=None):
        super().__init__("+CME ERROR {} (cmd={})".format(code, cmd))
        self.code = code
        self.cmd = cmd


class CMSError(ATError):
    """+CMS ERROR - message service error. `code` is the numeric reason."""

    def __init__(self, code, cmd=None):
        super().__init__("+CMS ERROR {} (cmd={})".format(code, cmd))
        self.code = code
        self.cmd = cmd


class NotSupportedError(ATError):
    """Command absent from this firmware build - not a transient failure."""


class SimLockedError(ATError):
    """SIM present but PIN-protected."""


# --------------------------------------------------------------------------
# Input validation
#
# AT commands are a text protocol over a serial line. An unvalidated value
# containing CR, LF or a quote would let a caller append arbitrary commands to
# whatever the caller thought was a single request. Phone numbers, APNs and
# hostnames are therefore whitelisted, never merely escaped.
# --------------------------------------------------------------------------

_ALLOWED_APN = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"
                   "0123456789._-")


def _clean_number(raw):
    """Accept '+62 812-3456' style input; return '+628123456' or raise."""
    s = str(raw).strip()
    out = []
    for ch in s:
        if ch.isdigit():
            out.append(ch)
        elif ch == "+" and not out:
            out.append("+")
        elif ch in " -()":
            continue
        else:
            raise ValueError("illegal character in phone number: {!r}".format(ch))
    num = "".join(out)
    digits = num.lstrip("+")
    if not 7 <= len(digits) <= 15:
        raise ValueError("phone number must be 7-15 digits, got {}".format(len(digits)))
    return num


def _clean_apn(raw):
    apn = str(raw).strip()
    if not apn or len(apn) > 64:
        raise ValueError("APN must be 1-64 characters")
    bad = set(apn) - _ALLOWED_APN
    if bad:
        raise ValueError("illegal character(s) in APN: {}".format(sorted(bad)))
    return apn


def _clean_host(raw):
    """Host for AT+CIPSTART / AT+CDNSGIP. Digits, letters, dots, colons."""
    host = str(raw).strip()
    if not host or len(host) > 255:
        raise ValueError("host must be 1-255 characters")
    bad = set(host) - _ALLOWED_APN - set("-:")
    if bad:
        raise ValueError("illegal character(s) in host: {}".format(sorted(bad)))
    return host


def _clean_text(raw, max_len=160):
    """GSM 7-bit SMS body. Control characters would confuse the TE parser."""
    text = str(raw)
    if not text:
        raise ValueError("message body is empty")
    if len(text) > max_len:
        raise ValueError("message is {} chars, limit {}".format(len(text), max_len))
    for ch in text:
        if ord(ch) < 0x20 or ord(ch) == 0x7F:
            raise ValueError("control character in message body: {:#04x}".format(ord(ch)))
    return text


# --------------------------------------------------------------------------
# Response
# --------------------------------------------------------------------------

class Response(object):
    """Payload lines of one command, without the terminator or URCs."""

    __slots__ = ("lines", "raw_urcs")

    def __init__(self, lines=(), raw_urcs=()):
        self.lines = list(lines)
        self.raw_urcs = list(raw_urcs)

    def __len__(self):
        return len(self.lines)

    def __iter__(self):
        return iter(self.lines)

    def __repr__(self):
        return "Response({!r})".format(self.lines)

    def first(self, prefix):
        """First line starting with `prefix`, or None."""
        for ln in self.lines:
            if ln.startswith(prefix):
                return ln
        return None

    def value(self, prefix, index=-1):
        """
        Field from the first `prefix` line. `index` 0 = the '+CSQ' token itself,
        -1 = last field. '+CSQ: 19,99' -> value('+CSQ', 1) == '19'.
        """
        ln = self.first(prefix)
        if ln is None:
            return None
        parts = ln.split(":", 1)[1].strip().split(",")
        return parts[index].strip().strip('"') if parts else None

    def fields(self, prefix):
        ln = self.first(prefix)
        if ln is None:
            return []
        return [p.strip().strip('"') for p in ln.split(":", 1)[1].split(",")]

    def bare(self):
        """
        First non-empty payload line, for action-form commands that echo a bare
        value with no '+CMD:' prefix once ATE0 is in effect - AT+CGMI, AT+CGMM,
        AT+CGSN, AT+CIMI. These replies carry leading blank lines.
        """
        for ln in self.lines:
            if ln.strip():
                return ln.strip()
        return None

    def bare_lines(self, prefix=None):
        """Payload lines, optionally only those beginning with `prefix`."""
        if prefix is None:
            return list(self.lines)
        return [ln for ln in self.lines if ln.startswith(prefix)]


# --------------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------------

class Sim7600(object):
    """
    AT driver. Give it a configured machine.UART already wired to the module's
    UART pins at 115200 8N1.

        uart = UART(1, baudrate=115200, tx=4, rx=5, timeout=20)
        modem = Sim7600(uart)
        modem.begin()
        modem.sms_send("+6281234567890", "pump 3 fired")

    `on_urc` is called with each unsolicited line seen while a command is in
    flight, e.g. '+CMTI: "SM",4' when a text message lands.
    """

    def __init__(self, uart, timeout=3000, retries=2, on_urc=None, debug=False):
        self._uart = uart
        self.timeout = timeout
        self.retries = retries
        self.debug = debug
        self.on_urc = on_urc
        self._rx = bytearray()      # line buffer, survives across commands
        self._chunk = bytearray(64)
        self._urcs = []
        self._caps = None

    # -- transport ---------------------------------------------------------

    def _write_raw(self, data):
        if isinstance(data, str):
            data = data.encode()
        self._uart.write(data)
        self._uart.flush() if hasattr(self._uart, "flush") else None

    def _readline(self, deadline):
        """
        One CRLF-stripped line, or None at the deadline. Partial lines stay in
        self._rx so a value split across two reads is reassembled, and so a
        URC that lands mid-command is not thrown away with the buffer.
        """
        while True:
            nl = self._rx.find(b"\n")
            if nl >= 0:
                line = bytes(self._rx[:nl])
                del self._rx[:nl + 1]
                return line.decode("utf-8", "replace").strip("\r\x00 ")
            if ticks_diff(ticks_ms(), deadline) >= 0:
                return None
            try:
                n = self._uart.readinto(self._chunk)
            except OSError as err:
                # UART gone or wedged. Surface it as a timeout so callers see
                # one failure type instead of a platform-specific exception.
                raise ATTimeout("uart read failed: {}".format(err))
            if n:
                self._rx.extend(self._chunk[:n])

    def _classify(self, line):
        """-> ('ok'|'err'|'urc'|'data', payload)"""
        up = line.upper()
        if up == _TERMINAL_OK:
            return "ok", None
        for e in _TERMINAL_ERR:
            if up.startswith(e):
                return "err", line
        if line.startswith("+"):
            for p in _URC_PREFIXES:
                if up.startswith(p.upper()):
                    return "urc", line
        return "data", line

    def _exchange(self, cmd, timeout=None, expect=None):
        """
        Send `cmd`, read to its terminator (or to the `expect` token), return a
        Response. Retries transient timeouts; a real ERROR is raised at once
        because retrying it would just burn time on a deterministic failure.
        """
        timeout = self.timeout if timeout is None else timeout
        last = None
        for attempt in range(self.retries + 1):
            if self.debug:
                print(">>", cmd)
            try:
                self._write_raw(cmd + "\r\n")
            except OSError as err:
                raise ATTimeout("uart write failed: {}".format(err))
            try:
                return self._read_to_end(timeout, expect, cmd)
            except ATTimeout as err:
                last = err
                if self.debug:
                    print("!! timeout on", cmd)
                if attempt == self.retries:
                    break
                # A timed-out command leaves the module mid-reply; a bare CR
                # resynchronises the stream before the retry. If the port is
                # genuinely gone this write fails too, and retrying cannot
                # help - report the original timeout rather than masking it
                # with a transport error.
                sleep_ms(20)
                self._rx.clear()
                try:
                    self._uart.write(b"\r\n")
                except OSError:
                    raise last
                sleep_ms(50)
                self._rx.clear()
                sleep_ms(100 * (attempt + 1))
        raise last

    def _read_to_end(self, timeout, expect, cmd):
        deadline = ticks_ms() + timeout
        payload = []
        urcs = []
        while True:
            line = self._readline(deadline)
            if line is None:
                raise ATTimeout("no result code for {!r}".format(cmd))
            if self.debug:
                print("<<", line)
            if expect is not None and line.strip() == expect:
                return Response(payload, urcs)
            kind, value = self._classify(line)
            if kind == "ok":
                return Response(payload, urcs)
            if kind == "urc":
                urcs.append(value)
                self._urcs.append(value)
                if self.on_urc:
                    self.on_urc(value)
            elif kind == "err":
                up = value.upper()
                if up.startswith("+CME ERROR"):
                    raise CMError(_err_code(value), cmd)
                if up.startswith("+CMS ERROR"):
                    raise CMSError(_err_code(value), cmd)
                raise ATError("{} -> {}".format(cmd, value))
            else:
                payload.append(value)

    def at(self, cmd, timeout=None, expect=None):
        """Send a raw AT command. Use for anything this class does not wrap."""
        return self._exchange(cmd, timeout, expect)

    def try_at(self, cmd, timeout=None):
        """Run `cmd`, returning None instead of raising. For capability probes."""
        try:
            return self._exchange(cmd, timeout)
        except ATError:
            return None

    def write_then(self, cmd, payload, terminator="\x1a", timeout=20000):
        """
        For commands that take a raw body: issue `cmd`, wait for the module's
        prompt, then write `payload` + `terminator` (Ctrl-Z ends an SMS).
        """
        timeout = self.timeout if timeout is None else timeout
        self._write_raw(cmd + "\r\n")
        deadline = ticks_ms() + timeout
        # Phase 1: wait for the '>' prompt, discarding anything echoed.
        while True:
            line = self._readline(deadline)
            if line is None:
                if ticks_diff(ticks_ms(), deadline) >= 0:
                    raise ATTimeout("no prompt for {!r}".format(cmd))
                continue
            if self.debug:
                print("<<", line)
            if line.strip() == ">":
                break
            kind, _ = self._classify(line)
            if kind == "err":
                raise ATError("{} -> {}".format(cmd, line))
        # Phase 2: body plus terminator.
        self._write_raw(payload + terminator)
        # Phase 3: normal terminating result code.
        return self._read_to_end(timeout, None, cmd)

    def poll_urcs(self, idle_ms=200):
        """
        Drain unsolicited lines that arrived while no command was running.
        Returns the lines collected. Call from the main loop.
        """
        seen = list(self._urcs)
        self._urcs = []
        end = ticks_ms() + idle_ms
        while ticks_diff(ticks_ms(), end) < 0:
            line = self._readline(end)
            if line is None:
                continue
            kind, value = self._classify(line)
            if kind == "urc":
                seen.append(value)
                if self.on_urc:
                    self.on_urc(value)
            elif kind == "data":
                # Payload outside any command - nothing sensible to do with it.
                if self.debug:
                    print("<< stray", value)
        return seen

    def drain(self):
        """Throw away buffered input. After a baud change or a hard reset."""
        self._rx.clear()
        self._urcs = []

    # -- bring-up ----------------------------------------------------------

    def begin(self, apn=None, ctx=1):
        """
        Standard bring-up: echo off, verbose errors, SMS in text mode, new-SMS
        notification. Returns self so it can chain.

        Set `apn` to force a PDP context; None leaves whatever the SIM already
        negotiated (correct for Telkomsel, which pushes cmnet itself).
        """
        self.drain()
        self.at("AT")
        self.at("ATE0")
        self.at("AT+CMEE=2")          # numeric +CME/+CMS errors
        self.at("AT+CMGF=1")          # SMS text mode
        self.at('AT+CSCS="GSM"')
        self.at("AT+CNMI=2,1")        # route new-SMS to the AT port as +CMTI
        self.at("AT+CSCLK=0")         # never sleep out from under us
        if apn is not None:
            self.set_apn(apn, ctx)
        return self

    def sync(self, attempts=20, delay_ms=500):
        """Block until the module answers AT. Use after PWRKEY or a reset."""
        for _ in range(attempts):
            try:
                if self.at("AT", timeout=1000):
                    return True
            except ATError:
                self.drain()
                sleep_ms(delay_ms)
        return False

    # -- identity ----------------------------------------------------------

    def info(self):
        return {
            "manufacturer": self.at("AT+CGMI").bare(),
            "model": self.at("AT+CGMM").bare(),
            "firmware": self.at("AT+CGMR").value("+CGMR"),
        }

    def imei(self):
        return self.at("AT+CGSN").bare()

    def iccid(self):
        return self.at("AT+CIMI").bare()

    def sim_status(self):
        """'READY', 'SIM PIN', ... """
        return self.at("AT+CPIN?").value("+CPIN")

    def unlock(self, pin):
        """Unblock a PIN-protected SIM. `pin` must be 4-8 digits."""
        pin = str(pin)
        if not pin.isdigit() or not 4 <= len(pin) <= 8:
            raise ValueError("SIM PIN must be 4-8 digits")
        return self.at('AT+CPIN="{}"'.format(pin))

    def version(self):
        """AT+CSUB - hardware stack + full build id."""
        r = self.at("AT+CSUB")
        return [v.strip('"') for v in r.fields("+CSUB")]

    # -- network -----------------------------------------------------------

    def signal(self):
        """
        (rssi_dbm, ber) from AT+CSQ. rssi_dbm is None when the modem reports
        99, which means 'unknown' rather than 'no signal'.
        """
        raw = self.at("AT+CSQ").value("+CSQ", 0)
        rssi = int(raw)
        ber = int(self.at("AT+CSQ").value("+CSQ", 1))
        dbm = None if rssi == 99 else -113 + 2 * rssi
        return dbm, ber

    def operator(self):
        """
        (name, access_technology, status), e.g. ('Telkomsel', 'LTE', '0').

        AT+COPS=0,2 asks for the long alphanumeric format, but this firmware
        accepts the command and then ignores it, replying in short format
        anyway - so there is no switch to expose. The network name arrives as
        text either way; only the technology is numeric, decoded via
        TECH_NAMES.
        """
        f = self.at("AT+COPS?").fields("+COPS")
        if len(f) < 4:
            return (None, None, f[0] if f else None)
        return (f[2], TECH_NAMES.get(f[3], f[3]), f[0])

    def registration(self):
        """
        CEREG stat as (urc_mode, stat). stat: 1 home, 5 roaming,
        0 not registered, 2 searching, 3 denied, 4 unknown.
        """
        f = self.at("AT+CEREG?").fields("+CEREG")
        return (int(f[0]), int(f[1])) if len(f) >= 2 else (0, 0)

    def is_registered(self):
        return self.registration()[1] in (1, 5)

    def system_info(self):
        """AT+CPSI - serving system, first field is 'LTE,Online,MCC-MNC,...'."""
        return self.at("AT+CPSI?").value("+CPSI", 0)

    def network_mode(self):
        return int(self.at("AT+CNMP?").value("+CNMP"))

    def attached(self):
        return self.at("AT+CGATT?").value("+CGATT") == "1"

    def set_apn(self, apn, ctx=1):
        return self.at('AT+CGDCONT={},"IP","{}"'.format(ctx, _clean_apn(apn)))

    def get_apn(self, ctx=1):
        """
        APN of one PDP context. AT+CGDCONT? returns one '+CGDCONT:' line per
        context, so this has to walk lines rather than a flattened field list.
        """
        r = self.at("AT+CGDCONT?")
        for ln in r.bare_lines("+CGDCONT"):
            f = [p.strip().strip('"') for p in ln.split(":", 1)[1].split(",")]
            if len(f) >= 3 and f[0] == str(ctx):
                return f[2]
        return None

    def ip_address(self, ctx=1):
        """
        Local PDP address. Uses AT+CGPADDR, not AT+CIFSR: CIFSR returns ERROR
        on this firmware even while the context is up.
        """
        for ln in self.at("AT+CGPADDR").bare_lines("+CGPADDR"):
            f = [p.strip().strip('"') for p in ln.split(":", 1)[1].split(",")]
            if len(f) >= 2 and f[0] == str(ctx):
                return None if f[1] in ("0.0.0.0", "") else f[1]
        return None

    def resolve(self, name):
        """AT+CDNSGIP - cellular DNS. Returns an IPv4 string."""
        return self.at('AT+CDNSGIP="{}"'.format(_clean_host(name)),
                       timeout=15000).value("+CDNSGIP", 2)

    # -- SMS ---------------------------------------------------------------

    def sms_center(self):
        return self.at("AT+CSCA?").value("+CSCA", 0)

    def sms_status(self):
        """(used, capacity) for the first SMS store."""
        f = self.at("AT+CPMS?").fields("+CPMS")
        return (int(f[1]), int(f[2])) if len(f) >= 3 else (0, 0)

    def sms_list(self, folder="REC READ"):
        """
        [(index, status, sender, timestamp)] for one store. Valid folders:
        REC READ, REC UNREAD, STO UNSENT, STO SENT, ALL.
        """
        if folder not in ("REC READ", "REC UNREAD", "STO UNSENT", "STO SENT", "ALL"):
            raise ValueError("unknown SMS folder: {!r}".format(folder))
        r = self.at('AT+CMGL="{}"'.format(folder), timeout=8000)
        out = []
        for ln in r.lines:
            if ln.startswith("+CMGL:"):
                f = [p.strip().strip('"') for p in ln.split(":", 1)[1].split(",")]
                out.append({"index": int(f[0]), "status": f[1],
                            "from": f[2], "time": f[4] if len(f) > 4 else ""})
        return out

    def sms_read(self, index):
        """{'status':..., 'from':..., 'time':..., 'text':...} for one message."""
        r = self.at("AT+CMGR={}".format(int(index)), timeout=8000)
        hdr = r.first("+CMGR")
        f = [p.strip().strip('"') for p in hdr.split(":", 1)[1].split(",")]
        # Body is everything after the header line. An SMS body beginning with
        # a bare 'OK' would be mistaken for a terminator; that is a known limit
        # of the chat protocol, not of this parser.
        at = r.lines.index(hdr)
        return {
            "status": f[0] if len(f) > 0 else "",
            "from": f[1] if len(f) > 1 else "",
            "time": f[3] if len(f) > 3 else "",
            "text": "\n".join(r.lines[at + 1:]),
        }

    def sms_send(self, number, text):
        """
        Send a text message, returns the module's message reference.
        One segment is 160 GSM-7 characters; longer is rejected rather than
        silently split, because the concatenation path is untested on this build.
        """
        number = _clean_number(number)
        text = _clean_text(text, 160)
        r = self.write_then('AT+CMGS="{}"'.format(number), text + "\r")
        ref = r.value("+CMGS")
        return int(ref) if ref is not None else None

    def sms_delete(self, index):
        return self.at("AT+CMGD={}".format(int(index)), timeout=8000)

    def sms_delete_all(self):
        for msg in self.sms_list("ALL"):
            self.sms_delete(msg["index"])
        return self.sms_status()

    # -- GNSS --------------------------------------------------------------
    # This board uses AT+CGPS, not AT+CGNSPWR.

    def gnss_on(self, nmea_port=None):
        """
        Start GNSS. `nmea_port` 0..3 selects the UART that carries NMEA:
        0 = USB NMEA, 1 = UART1, 2 = UART2, 3 = the AT port itself.

        Do not use port 3 while the AT driver owns that port - the module
        starts emitting $GPGGA into the command stream and every subsequent
        read times out. Waveshare documents port 3 only for boards that emit
        nothing otherwise; on this module port 1 works and port 3 does not.
        """
        if nmea_port == 3:
            raise ValueError(
                "nmea_port=3 puts NMEA on the AT port and will wedge the "
                "command channel; use 0, 1 or 2")
        if nmea_port is not None:
            self.at("AT+CGPSNMEAPORTCFG={}".format(int(nmea_port)))
        return self.at("AT+CGPS=1")

    def gnss_off(self, settle_ms=4000):
        """
        Stop GNSS and wait for the receiver to actually stop, so an immediate
        gnss_status() does not read stale 'on'. Pass settle_ms=0 to skip.
        """
        self.at("AT+CGPS=0")
        if settle_ms:
            sleep_ms(settle_ms)
        return self

    def gnss_status(self):
        """
        (on, fix_mode) from AT+CGPS?.

        The receiver takes a few seconds to wind down after AT+CGPS=0, and the
        module emits a bare '+CGPS: 0' in the meantime, so a status read too
        soon reports the old state. Pass settle_ms to wait it out.
        """
        f = self.at("AT+CGPS?").fields("+CGPS")
        return (int(f[0]), int(f[1])) if len(f) >= 2 else (int(f[0]) if f else 0, 0)

    # -- power -------------------------------------------------------------

    def reboot(self, wait=True, timeout_ms=30000):
        """AT+CRESET. With wait=True, block until the module answers again."""
        try:
            self.at("AT+CRESET", timeout=5000)
        except ATError:
            pass          # the module usually drops the port before answering
        self._rx.clear()
        self._urcs = []
        if wait and not self.sync(attempts=int(timeout_ms // 500)):
            raise ATTimeout("module did not come back after AT+CRESET")
        return self

    def radio_off(self):
        return self.at("AT+CFUN=0")

    def radio_on(self, wait=True):
        self.at("AT+CFUN=1", timeout=10000)
        return self.sync() if wait else self

    def sleep_mode(self, on):
        """AT+CSCLK. Only useful with a battery; the module must be >40 mA."""
        return self.at("AT+CSCLK={}".format(1 if on else 0))

    def set_baud(self, baud):
        """AT+IPR - takes effect after AT+CRESET."""
        return self.at("AT+IPR={}".format(int(baud)))

    def pwrkey(self, pulse_ms=1500):
        """
        Drive PWRKEY low. Requires a caller-held N-MOSFET; there is no AT way
        around the physical key.
        """
        raise NotImplementedError(
            "PWRKEY is a physical pin - pulse it with a GPIO/N-MOSFET, "
            "do not route it through this driver")

    # -- capabilities ------------------------------------------------------

    def capabilities(self):
        """
        Probe what this firmware actually implements. Cached after the first
        call. This build reports sms and dns True, tcp/http/mqtt False.
        """
        if self._caps is not None:
            return self._caps
        caps = {}
        for name, cmd in (
            ("sms", "AT+CMGF=1"),
            ("dns", 'AT+CDNSGIP="example.com"'),
            ("gnss", "AT+CGPS?"),
            ("tcp", "AT+CIPSOCK=6"),
            ("http", "AT+CHTTPCREATE"),
            ("mqtt", "AT+CMQNEW?"),
        ):
            caps[name] = self.try_at(cmd, timeout=15000) is not None
        # Leave no socket or PDP context behind from the probes.
        if caps["tcp"]:
            self.try_at("AT+CIPCLOSE=6")
        self._caps = caps
        return caps

    def require(self, feature):
        """Raise unless `feature` is supported. Guard before using tcp/http."""
        caps = self.capabilities()
        if not caps.get(feature):
            raise NotSupportedError(
                "{} is not available on this firmware build".format(feature))
        return True


def _err_code(line):
    """'+CME ERROR: 10' -> '10'. Non-numeric text passes through."""
    _, _, tail = line.partition(":")
    return tail.strip() or "unknown"