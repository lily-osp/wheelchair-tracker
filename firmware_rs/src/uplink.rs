//! HTTP uplink through the modem's own `AT+HTTP` stack.
//!
//! PPP (`ATD*99***1#`) is not usable on this carrier/firmware combination:
//! the TCP handshake completes but the first request is reset before any
//! byte comes back, verified on hardware 2026-11-05. `AT+HTTP` runs inside
//! the module instead, needs no DNS in AT context, and reports its result as
//! `+HTTPACTION: <method>,<status>,<len>` - which is exactly what a tracker
//! needs and is observable from the UART.
//!
//! The key travels as the `X-AIO-Key` query parameter because `AT+HTTPPARA`
//! cannot set arbitrary headers; plain HTTP means it is still in the clear.

use core::fmt::Write as _;

use embassy_time::Duration;
use heapless::String;

use crate::at::field;
use crate::sim7600::{ModemError, Sim7600};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UplinkError {
    /// `AT+HTTPINIT` or a parameter was rejected.
    Init,
    /// The URL failed local validation (quote/newline injection guard).
    BadUrl,
    /// `AT+HTTPDATA` never reached the DOWNLOAD prompt.
    Data,
    /// No `+HTTPACTION` before the deadline.
    Timeout,
    /// HTTP status outside 2xx.
    Status(u16),
    /// Modem-side error on the AT channel.
    Rejected(ModemError),
}

impl From<ModemError> for UplinkError {
    fn from(e: ModemError) -> Self {
        Self::Rejected(e)
    }
}

/// Posts `body` to an absolute `http://` URL and returns the HTTP status.
/// Leaves the HTTP service terminated, so the next call starts clean.
pub async fn post_json(sim: &mut Sim7600<'_>, url: &str, body: &[u8]) -> Result<u16, UplinkError> {
    if !valid_url(url) || body.is_empty() {
        return Err(UplinkError::BadUrl);
    }

    // A session left open by a previous wake makes HTTPINIT fail. Recover
    // once - this is the documented behaviour, not a fault.
    if !sim.ok("AT+HTTPINIT").await {
        let _ = sim.ok("AT+HTTPTERM").await;
        if !sim.ok("AT+HTTPINIT").await {
            return Err(UplinkError::Init);
        }
    }

    let status = post_inner(sim, url, body).await;
    let _ = sim.ok("AT+HTTPTERM").await;
    status
}

async fn post_inner(sim: &mut Sim7600<'_>, url: &str, body: &[u8]) -> Result<u16, UplinkError> {
    let _ = sim.ok("AT+HTTPPARA=\"CID\",1").await;

    let mut cmd = String::<384>::new();
    let _ = write!(cmd, "AT+HTTPPARA=\"URL\",\"{}\"", url);
    if !sim.ok(cmd.as_str()).await {
        return Err(UplinkError::Init);
    }
    // "CONTENT", not "CONTENTTYPE": the latter is ERROR on LE20B05SIM7600M22,
    // and without it Adafruit answers 406.
    let _ = sim.ok("AT+HTTPPARA=\"CONTENT\",\"application/json\"").await;
    let _ = sim.ok("AT+HTTPPARA=\"ACCEPT\",\"application/json\"").await;

    let mut data = String::<64>::new();
    let _ = write!(data, "AT+HTTPDATA={},20000", body.len());
    let _ = sim.uart_mut().write(data.as_bytes());
    let _ = sim.uart_mut().write(b"\r\n");
    if sim.wait_line("DOWNLOAD", Duration::from_secs(5)).await.is_none() {
        return Err(UplinkError::Data);
    }

    let _ = sim.uart_mut().write(body);
    let _ = sim.uart_mut().write(b"\r\n");
    if sim.wait_line("OK", Duration::from_secs(10)).await.is_none() {
        return Err(UplinkError::Data);
    }

    let _ = sim.ok("AT+HTTPACTION=1").await;
    let deadline = Duration::from_millis(crate::config::UPLINK_DEADLINE_MS as u64);
    let Some(urc) = sim.wait_line("+HTTPACTION:", deadline).await else {
        return Err(UplinkError::Timeout);
    };
    // "+HTTPACTION: 1,200,195" -> status is field 1.
    let code: u16 = field(urc.as_str(), 1).parse().unwrap_or(0);
    if (200..300).contains(&code) {
        Ok(code)
    } else {
        Err(UplinkError::Status(code))
    }
}

/// Local guard: no quotes, CR/LF or spaces, `http://` only, sane length.
fn valid_url(url: &str) -> bool {
    !url.is_empty()
        && url.len() <= 300
        && url.starts_with("http://")
        && !url
            .bytes()
            .any(|b| matches!(b, b'"' | b'\r' | b'\n' | b' '))
}
