//! SIM7600CE-T AT transport, async.
//!
//! Ported from `Sim7600At.{h,cpp}`, which was verified against firmware
//! LE20B05SIM7600M22 (AT+CSUB: MDM9x07_LE20_S_22_V1.14_240221) over USB.
//! The pure parsing/classification/validation logic lives in `at.rs` and is
//! host-tested; this module is the timing/transport glue that needs a real
//! UART. Bench-checklist item before trusting it on hardware.
//!
//! Hardware notes baked in from that testing:
//!   - GNSS is AT+CGPS, NOT AT+CGNSPWR.
//!   - AT+CGPSNMEAPORTCFG=3 wedges the AT port. Use 1 (or skip NMEA - this
//!     firmware never reads NMEA).
//!   - AT+CIFSR is broken; use AT+CGPADDR.
//!   - With echo off, action-form commands return BARE values.
//!   - +CPIN/+CEREG must not be filtered as URCs.

use core::fmt::Write as _;

use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_io_async::{BufRead, Write};
use esp_hal::uart::Uart;
use heapless::{String, Vec as HVec};

use crate::at::{
    AtResult, LINE_CAP, LineKind, LineBuffer, classify, cme_code, field, find, first_non_empty,
    valid_apn, valid_body, valid_number,
};

pub const MAX_LINES: usize = 8;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModemError {
    Timeout,
    Rejected,
    Uart,
    /// Value rejected by local validation (line protocol injection guard).
    InvalidValue,
}

pub struct CommandOutput {
    pub res: AtResult,
    pub lines: HVec<String<LINE_CAP>, MAX_LINES>,
    pub urcs: HVec<String<LINE_CAP>, 4>,
}

impl CommandOutput {
    const fn empty() -> Self {
        Self {
            res: AtResult::EMPTY,
            lines: HVec::new(),
            urcs: HVec::new(),
        }
    }

    /// First line with `prefix`.
    pub fn line_with(&self, prefix: &str) -> Option<&str> {
        let strs: HVec<&str, MAX_LINES> =
            HVec::from_iter(self.lines.iter().map(|s| s.as_str()));
        find(strs.iter().copied(), prefix)
    }

    pub fn value(&self, prefix: &str, index: usize) -> String<64> {
        match self.line_with(prefix) {
            Some(l) => field(l, index),
            None => String::new(),
        }
    }

    /// First non-empty payload line - for bare-value action replies.
    pub fn bare_value(&self) -> Option<&str> {
        let strs: HVec<&str, MAX_LINES> =
            HVec::from_iter(self.lines.iter().map(|s| s.as_str()));
        first_non_empty(strs.iter().copied())
    }
}

pub struct Sim7600<'d> {
    uart: &'d mut Uart<'d, esp_hal::Async>,
    timeout: Duration,
    retries: u8,
    buf: LineBuffer,
}

impl<'d> Sim7600<'d> {
    pub fn new(uart: &'d mut Uart<'d, esp_hal::Async>, timeout: Duration, retries: u8) -> Self {
        let mut this = Self {
            uart,
            timeout,
            retries,
            buf: LineBuffer::new(),
        };
        this.drain();
        this
    }

    /// The UART, for the caller that hands it to the PPP runner.
    pub fn uart_mut(&mut self) -> &mut Uart<'d, esp_hal::Async> {
        &mut self.uart
    }

    /// Discards buffered input - after a baud change or a hard reset.
    pub fn drain(&mut self) {
        self.buf.reset();
        let mut sink = [0u8; 64];
        // `read` waits for data; only `read_buffered` drains without hanging.
        while self
            .uart
            .read_buffered(&mut sink)
            .map(|n| n > 0)
            .unwrap_or(false)
        {}
    }

    /// Sends one command and reads to the first terminal result code, with
    /// internal timeout-retry: a real ERROR will not fix itself, only a
    /// timeout is retried, with a bare CR to resync the stream (mirrors the
    /// C++ `command()`).
    pub async fn command(&mut self, cmd: &str, timeout: Option<Duration>) -> CommandOutput {
        let t = timeout.unwrap_or(self.timeout);
        for attempt in 0..=self.retries {
            let out = self.send_and_read(cmd, t, None).await;
            match out {
                Some(out) if !out.res.timed_out || attempt == self.retries => {
                    return out;
                }
                _ => {
                    // Timed out: the module is mid-reply. A bare CR resyncs.
                    Timer::after_millis(20).await;
                    self.buf.reset();
                    let _ = self.uart.write(b"\r\n");
                    self.buf.reset();
                    Timer::after_millis(100 * (attempt as u64 + 1)).await;
                }
            }
        }
        CommandOutput::empty()
    }

    pub async fn ok(&mut self, cmd: &str) -> bool {
        self.command(cmd, None).await.res.ok
    }

    /// Wait for the module to answer AT. Call after pulsing PWRKEY.
    pub async fn sync(&mut self, attempts: u8, delay_ms: u32) -> bool {
        for _ in 0..attempts {
            let out = self.send_and_read("AT", Duration::from_millis(1000), None).await;
            if out.is_some_and(|o| o.res.ok) {
                return true;
            }
            self.drain();
            Timer::after_millis(delay_ms as u64).await;
        }
        false
    }

    /// Bring-up: echo off, qualified errors, SMS text mode, never sleep.
    pub async fn bring_up(&mut self, apn: Option<&str>) -> bool {
        self.drain();
        if !self.ok("AT").await {
            return false;
        }
        for cmd in [
            "ATE0", // echo off: bare replies follow
            "AT+CMEE=2",
            "AT+CMGF=1",
            "AT+CSCS=\"GSM\"",
            "AT+CNMI=2,1", // new SMS as +CMTI on the AT port
            "AT+CSCLK=0",  // never sleep under us
        ] {
            if !self.ok(cmd).await {
                return false;
            }
        }
        match apn {
            Some(a) if !a.is_empty() => self.set_apn(a).await,
            _ => true,
        }
    }

    pub async fn set_apn(&mut self, apn: &str) -> bool {
        if !valid_apn(apn) {
            return false;
        }
        let mut cmd = String::<96>::new();
        let _ = write!(cmd, "AT+CGDCONT=1,\"IP\",\"{}\"", apn);
        self.ok(cmd.as_str()).await
    }

    /// RSSI in dBm, -999 when the modem reports unknown.
    pub async fn signal(&mut self) -> i32 {
        let out = self.command("AT+CSQ", None).await;
        if !out.res.ok {
            return -999;
        }
        let v: i32 = out.value("+CSQ", 0).parse().unwrap_or(99);
        if v == 99 {
            -999
        } else {
            -113 + 2 * v
        }
    }

    /// Registered: home (1) or roaming (5).
    pub async fn is_registered(&mut self) -> bool {
        let out = self.command("AT+CEREG?", None).await;
        if !out.res.ok {
            return false;
        }
        let stat: i32 = out.value("+CEREG", 1).parse().unwrap_or(0);
        stat == 1 || stat == 5
    }

    pub async fn is_attached(&mut self) -> bool {
        let out = self.command("AT+CGATT?", None).await;
        out.res.ok && out.value("+CGATT", 0) == "1"
    }

    /// SIM status, e.g. "READY".
    pub async fn sim_status(&mut self) -> String<32> {
        let v = self.command("AT+CPIN?", None).await.value("+CPIN", 0);
        let mut s = String::new();
        let _ = s.push_str(v.as_str());
        s
    }

    /// Manufacturer / model / firmware. With echo off these reply bare, so
    /// take the first non-empty payload line rather than a "+CGMR:"-style
    /// prefix.
    pub async fn info(&mut self) -> Option<(String<48>, String<32>, String<48>)> {
        let mfg = self.command("AT+CGMI", None).await;
        let mdl = self.command("AT+CGMM", None).await;
        let fw = self.command("AT+CGMR", None).await;
        if !(mfg.res.ok && mdl.res.ok && fw.res.ok) {
            return None;
        }
        let mut model = String::<32>::new();
        let _ = model.push_str(mdl.bare_value().unwrap_or(""));
        if model.is_empty() {
            return None;
        }
        // CGMR is prefixed ("+CGMR: ...") on this firmware; CGMI/CGMM are bare.
        let fw_str = fw.value("+CGMR", 0);
        let fw_out = if fw_str.is_empty() {
            fw.bare_value().unwrap_or("")
        } else {
            fw_str.as_str()
        };
        Some((str_s(mfg.bare_value().unwrap_or("")), model, str_s(fw_out)))
    }

    /// GNSS power with NMEA routed to the modem's own port (1) - never 3,
    /// which wedges the AT channel.
    pub async fn gnss_on(&mut self) -> bool {
        // Idempotent: on firmware LE20B05SIM7600M22, AT+CGPS=1 returns ERROR
        // when GNSS is already on, so probe first.
        let st = self.command("AT+CGPS?", None).await;
        if st.res.ok && st.value("+CGPS", 0) == "1" {
            return true;
        }
        if !self.ok("AT+CGPSNMEAPORTCFG=1").await {
            return false;
        }
        self.ok("AT+CGPS=1").await
    }

    pub async fn gnss_off(&mut self, settle_ms: u32) {
        let _ = self.ok("AT+CGPS=0").await;
        if settle_ms > 0 {
            Timer::after_millis(settle_ms as u64).await;
        }
    }

    /// Polls AT+CGNSSINFO (falls back to AT+CGPSINFO on ERROR; the exact
    /// capability set of this firmware build is unverified - issue #7).
    pub async fn position(&mut self) -> Option<crate::gpsinfo::Fix> {
        let out = self.command("AT+CGNSSINFO", None).await;
        if out.res.ok {
            if let Some(l) = out.line_with("+CGNSSINFO") {
                return crate::gpsinfo::parse_position_line(l);
            }
        }
        let out = self.command("AT+CGPSINFO", None).await;
        out.line_with("+CGPSINFO")
            .and_then(crate::gpsinfo::parse_position_line)
    }

    /// Sends one SMS; returns the message reference.
    pub async fn sms_send(&mut self, number: &str, text: &str) -> Result<u16, ModemError> {
        let Some(clean) = valid_number(number) else {
            return Err(ModemError::InvalidValue);
        };
        if !valid_body(text) {
            return Err(ModemError::InvalidValue);
        }

        let mut cmd = String::<48>::new();
        let _ = write!(cmd, "AT+CMGS=\"{}\"", clean);

        // Phase 1: send, wait for the '>' prompt.
        let _ = self.uart.write(cmd.as_bytes());
        let _ = self.uart.write(b"\r\n");
        let Some(_) = self.read_lines(Duration::from_millis(20_000), Some(">")).await else {
            return Err(ModemError::Timeout);
        };
        // Phase 2: body plus CR before Ctrl-Z; without the CR some firmware
        // includes it in the message.
        let _ = self.uart.write(text.as_bytes());
        let _ = self.uart.write(b"\r\x1a");
        // Phase 3: normal terminating result code.
        let Some(out) = self.read_lines(Duration::from_millis(20_000), None).await else {
            return Err(ModemError::Timeout);
        };
        if !out.res.ok {
            return Err(ModemError::Rejected);
        }
        Ok(out.value("+CMGS", 0).parse().unwrap_or(0))
    }

    /// Idempotent power-on probe: true when the module answers.
    pub async fn answers(&mut self) -> bool {
        self.send_and_read("AT", Duration::from_millis(700), None)
            .await
            .is_some_and(|o| o.res.ok)
    }

    /// Dials the PPP data connection; returns once the stream is binary.
    pub async fn dial(&mut self, timeout: Duration) -> Result<(), ModemError> {
        let _ = self.uart.write(b"ATD*99***1#\r\n");
        let deadline = Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.as_ticks() == 0 {
                return Err(ModemError::Timeout);
            }
            let mut chunk = [0u8; 128];
            match with_timeout(remaining, self.uart.read_async(&mut chunk)).await {
                Ok(Ok(n)) => {
                    let mut pending: [String<LINE_CAP>; MAX_LINES] =
                        core::array::from_fn(|_| String::new());
                    let got = self.buf.feed(&chunk[..n], &mut pending);
                    for line in &pending[..got] {
                        if line.eq_ignore_ascii_case("CONNECT") {
                            return Ok(());
                        }
                        if classify(line) == LineKind::Err {
                            return Err(ModemError::Rejected);
                        }
                    }
                }
                Ok(Err(_)) => return Err(ModemError::Uart),
                Err(_) => return Err(ModemError::Timeout),
            }
        }
    }

    async fn send_and_read(
        &mut self,
        cmd: &str,
        timeout: Duration,
        prompt: Option<&str>,
    ) -> Option<CommandOutput> {
        let _ = self.uart.write(cmd.as_bytes());
        let _ = self.uart.write(b"\r\n");
        self.read_lines(timeout, prompt).await
    }

    /// Reads lines until OK/ERROR/prompt/timeout, classifying as it goes.
    async fn read_lines(
        &mut self,
        timeout: Duration,
        prompt: Option<&str>,
    ) -> Option<CommandOutput> {
        let mut out = CommandOutput::empty();
        let mut pending: [String<LINE_CAP>; MAX_LINES] = core::array::from_fn(|_| String::new());
        let deadline = Instant::now() + timeout;

        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.as_ticks() == 0 {
                out.res.timed_out = true;
                out.res.ok = false;
                return Some(out);
            }

            let mut chunk = [0u8; 256];
            match with_timeout(remaining, self.uart.read_async(&mut chunk)).await {
                Ok(Ok(n)) => {
                    let got = self.buf.feed(&chunk[..n], &mut pending);
                    for line in &pending[..got] {
                        if prompt.is_some_and(|p| line == p) {
                            return Some(out);
                        }
                        match classify(line) {
                            LineKind::Ok => {
                                out.res.ok = true;
                                return Some(out);
                            }
                            LineKind::Err => {
                                if line.starts_with("+CME ERROR")
                                    || line.starts_with("+CMS ERROR")
                                {
                                    out.res.cme_code = cme_code(line);
                                }
                                out.res.ok = false;
                                return Some(out);
                            }
                            LineKind::Urc => {
                                let _ = out.urcs.push(line.clone());
                            }
                            LineKind::Data => {
                                let _ = out.lines.push(line.clone());
                            }
                        }
                    }
                }
                Ok(Err(_)) => {
                    out.res.ok = false;
                    return Some(out);
                }
                // Outer deadline - the reads above are bounded by the same
                // deadline, so this is the common timeout exit.
                Err(_) => {
                    out.res.timed_out = true;
                    out.res.ok = false;
                    return Some(out);
                }
            }
        }
    }
}

/// `BufRead` adapter over the esp-hal async UART: `Runner::run` wants
/// `fill_buf`/`consume`, which the UART does not provide natively.
pub struct PppUart<'a, 'd> {
    uart: &'a mut Uart<'d, esp_hal::Async>,
    buf: [u8; 512],
    pos: usize,
    len: usize,
}

impl<'a, 'd> PppUart<'a, 'd> {
    pub fn new(uart: &'a mut Uart<'d, esp_hal::Async>) -> Self {
        Self {
            uart,
            buf: [0; 512],
            pos: 0,
            len: 0,
        }
    }
}

impl embedded_io::ErrorType for PppUart<'_, '_> {
    type Error = core::convert::Infallible;
}

impl embedded_io_async::Read for PppUart<'_, '_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // Drain the internal fill buffer first, then the UART.
        if self.pos < self.len {
            let n = (self.len - self.pos).min(buf.len());
            buf[..n].copy_from_slice(&self.buf[self.pos..self.pos + n]);
            self.pos += n;
            return Ok(n);
        }
        Ok(self.uart.read_async(buf).await.unwrap_or(0))
    }
}

impl BufRead for PppUart<'_, '_> {
    async fn fill_buf(&mut self) -> Result<&[u8], Self::Error> {
        if self.pos == self.len {
            self.pos = 0;
            self.len = match self.uart.read_async(&mut self.buf).await {
                Ok(n) => n,
                Err(_) => 0,
            };
            if self.len == 0 {
                // Modem dropped the port: report EOF via an empty fill.
                return Ok(&[]);
            }
        }
        Ok(&self.buf[self.pos..self.len])
    }

    fn consume(&mut self, amt: usize) {
        self.pos = (self.pos + amt).min(self.len);
    }
}

impl Write for PppUart<'_, '_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        // write_all semantics are required by the PPP protocol; the UART
        // write_async buffers internally.
        match self.uart.write_async(buf).await {
            Ok(_) => Ok(buf.len()),
            Err(_) => Ok(0),
        }
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        let _ = self.uart.flush_async().await;
        Ok(())
    }
}

fn str_s(s: &str) -> String<48> {
    let mut out = String::new();
    let _ = out.push_str(s);
    out
}
