//! Pure AT line-protocol core: buffering, line classification, field
//! extraction and validation. No I/O, no clocks - host-testable.
//!
//! Firmware quirks baked in from bench verification against the live modem
//! (firmware LE20B05SIM7600M22; see hardware.md and firmware_rs/README.md):
//!   - With echo off, action-form commands return BARE values (AT+CGMM ->
//!     "SIM7600CE-T"): no "+CGMM:" prefix, leading blank lines. A parser that
//!     only looks for prefixed lines silently returns nothing.
//!   - +CPIN/+CEREG share a prefix with their URCs and must NOT be filtered
//!     as unsolicited, or the reply to the query that asked for them is
//!     discarded.
//!   - Plain ERROR is the base form; +CME/+CMS ERROR are qualified variants.

use heapless::String;

pub const MAX_LINES: usize = 8;
pub const LINE_CAP: usize = 192;
pub const RX_CAP: usize = 1024;

/// Unrequested result codes. Deliberately narrow: query responses sharing a
/// prefix with a URC (+CPIN, +CEREG, +CREG, +CGREG, +CMSS, +CDEL) are absent
/// on purpose.
const URC_PREFIXES: [&str; 11] = [
    "+CMTI", "+CMT", "+CUSD", "+CGEV", "+CIEV", "+CRING", "+RING", "+POLL",
    "+PBREADY", "+CTS", "+CERR",
];

/// "ERROR" first: plain ERROR is the base form, +CME/+CMS are qualified
/// variants matched by prefix.
const TERMINAL_ERR: [&str; 6] =
    ["ERROR", "+CME ERROR", "+CMS ERROR", "NO CARRIER", "ABORTED", "BUSY"];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LineKind {
    Ok,
    Err,
    Urc,
    Data,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AtResult {
    pub ok: bool,
    pub timed_out: bool,
    /// Numeric reason from +CME ERROR, 0 if none.
    pub cme_code: u8,
    pub count: usize,
}

impl AtResult {
    pub const EMPTY: Self = Self {
        ok: false,
        timed_out: false,
        cme_code: 0,
        count: 0,
    };
}

/// Byte-to-line splitter with overflow accounting. A runaway stream must not
/// be allowed to drop bytes silently: a corrupted line is worse than a
/// missing one, so an overflowing line is poisoned (never emitted) and its
/// loss counted once.
#[derive(Debug)]
pub struct LineBuffer {
    rx: [u8; RX_CAP],
    len: usize,
    poisoned: bool,
    dropped: u32,
}

impl LineBuffer {
    pub const fn new() -> Self {
        Self {
            rx: [0; RX_CAP],
            len: 0,
            poisoned: false,
            dropped: 0,
        }
    }

    pub fn reset(&mut self) {
        self.len = 0;
        self.poisoned = false;
        self.dropped = 0;
    }

    pub fn dropped(&self) -> u32 {
        self.dropped
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0 && !self.poisoned
    }

    /// Feeds bytes and copies out every complete line found. Lines are
    /// trimmed; empty lines and overflowed lines are never emitted.
    pub fn feed(&mut self, bytes: &[u8], out: &mut [String<LINE_CAP>]) -> usize {
        let mut produced = 0;
        for &b in bytes {
            if b == b'\n' {
                if self.poisoned {
                    self.dropped += 1;
                } else if produced < out.len() {
                    let line = trim_ascii(&self.rx[..self.len]);
                    out[produced].clear();
                    if line.len() > LINE_CAP {
                        self.dropped += 1;
                    } else if !line.is_empty() {
                        let _ = out[produced].push_str(line);
                        produced += 1;
                    }
                }
                self.len = 0;
                self.poisoned = false;
            } else if self.len < RX_CAP {
                self.rx[self.len] = b;
                self.len += 1;
            } else {
                self.poisoned = true;
            }
        }
        produced
    }
}

pub fn trim_ascii(s: &[u8]) -> &str {
    let mut a = 0;
    let mut b = s.len();
    while a < b && (s[a] as char).is_ascii_whitespace() {
        a += 1;
    }
    while b > a && (s[b - 1] as char).is_ascii_whitespace() {
        b -= 1;
    }
    core::str::from_utf8(&s[a..b]).unwrap_or("")
}

/// Classifies one line.
pub fn classify(line: &str) -> LineKind {
    if line.eq_ignore_ascii_case("OK") {
        return LineKind::Ok;
    }
    for p in TERMINAL_ERR {
        if line.len() >= p.len() && line[..p.len()].eq_ignore_ascii_case(p) {
            return LineKind::Err;
        }
    }
    if line.starts_with('+') {
        for p in URC_PREFIXES {
            if line.starts_with(p) {
                return LineKind::Urc;
            }
        }
    }
    LineKind::Data
}

/// Numeric +CME ERROR code from a qualified error line, 0 if not parseable.
pub fn cme_code(line: &str) -> u8 {
    let Some(colon) = line.find(':') else {
        return 0;
    };
    let rest = line[colon + 1..].trim();
    rest.parse::<u8>().unwrap_or(0)
}

/// Extracts comma field `index` from a "+CMD: a,b,c" style line, quotes
/// stripped. Negative-style indexing is not needed; callers pass 0-based.
pub fn field(line: &str, index: usize) -> String<64> {
    let mut result = String::new();
    let Some(colon) = line.find(':') else {
        return result;
    };
    let body = line[colon + 1..].trim_start();
    let mut start = 0usize;
    let mut i = 0usize;
    while i < 12 {
        match body[start..].find(',') {
            Some(rel) => {
                let end = start + rel;
                if i == index {
                    return unquote(body[start..end].trim());
                }
                start = end + 1;
                i += 1;
            }
            None => {
                if i == index {
                    result = unquote(body[start..].trim());
                }
                break;
            }
        }
    }
    result
}

fn unquote(s: &str) -> String<64> {
    let mut out = String::new();
    for c in s.chars() {
        if c != '"' && out.push(c).is_err() {
            break;
        }
    }
    out
}

/// Finds the first line with the given prefix.
pub fn find<'a, I: IntoIterator<Item = &'a str>>(lines: I, prefix: &str) -> Option<&'a str> {
    lines
        .into_iter()
        .find(|l| l.len() >= prefix.len() && l.starts_with(prefix))
}

/// Extracts the first non-empty payload line (bare-value replies).
pub fn first_non_empty<'a, I: IntoIterator<Item = &'a str>>(lines: I) -> Option<&'a str> {
    lines.into_iter().find(|l| !l.is_empty())
}

/// AT is a line protocol: an unvalidated value containing CR, LF or a quote
/// would let a caller append arbitrary commands. These whitelist rather than
/// escape.
pub fn valid_number(number: &str) -> Option<String<24>> {
    let mut clean = String::new();
    for c in number.chars() {
        if c.is_ascii_digit() {
            let _ = clean.push(c);
        } else if c == '+' && clean.is_empty() {
            let _ = clean.push(c);
        } else if c == ' ' || c == '-' || c == '(' || c == ')' {
            continue;
        } else {
            return None;
        }
    }
    let digits = clean.trim_start_matches('+');
    if !(7..=15).contains(&digits.len()) {
        return None;
    }
    Some(clean)
}

pub fn valid_apn(apn: &str) -> bool {
    !(apn.is_empty() || apn.len() > 64)
        && apn.chars()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '.' | '_' | '-'))
}

pub fn valid_host(host: &str) -> bool {
    !(host.is_empty() || host.len() > 255)
        && host
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '.' | '-' | ':'))
}

/// One GSM-7 segment: 160 chars, no control characters.
pub fn valid_body(text: &str) -> bool {
    !text.is_empty()
        && text.len() <= 160
        && text
            .chars()
            .all(|c| c.is_ascii_graphic() || c == ' ')
}

#[cfg(test)]
mod tests {
    use super::*;
    use heapless::Vec as HVec;

    fn lines() -> [String<LINE_CAP>; MAX_LINES] {
        core::array::from_fn(|_| String::new())
    }

    #[test]
    fn feed_splits_and_trims_lines() {
        let mut lb = LineBuffer::new();
        let mut out = lines();
        let n = lb.feed(b"\r\nOK\r\nAT+CGMM\r\nSIM7600CE-T\r\n", &mut out);
        assert_eq!(n, 3, "leading empty line dropped, 3 real lines");
        assert_eq!(out[0].as_str(), "OK");
        assert_eq!(out[2].as_str(), "SIM7600CE-T");
    }

    #[test]
    fn feed_counts_overflow_not_silently_truncating() {
        let mut lb = LineBuffer::new();
        let mut out = lines();
        // One overflowing line, properly newline-terminated.
        let mut long = [b'A'; RX_CAP + 10];
        long[RX_CAP + 9] = b'\n';
        let n = lb.feed(&long, &mut out);
        assert_eq!(n, 0, "overflowed line not emitted");
        assert_eq!(lb.dropped(), 1);
        // A following clean line still parses.
        let n = lb.feed(b"OK\n", &mut out);
        assert_eq!(n, 1);
        assert_eq!(out[0].as_str(), "OK");
    }

    #[test]
    fn classify_finds_ok_errors_urcs_and_data() {
        assert_eq!(classify("OK"), LineKind::Ok);
        assert_eq!(classify("ok"), LineKind::Ok);
        assert_eq!(classify("ERROR"), LineKind::Err);
        assert_eq!(classify("+CME ERROR: 4"), LineKind::Err);
        assert_eq!(classify("+CMS ERROR: 300"), LineKind::Err);
        assert_eq!(classify("NO CARRIER"), LineKind::Err);
        assert_eq!(classify("+CMTI: \"SM\",3"), LineKind::Urc);
        assert_eq!(classify("+CGMR: LE20B05"), LineKind::Data);
        assert_eq!(classify("SIM7600CE-T"), LineKind::Data);
    }

    #[test]
    fn classify_does_not_filter_query_sharing_urc_prefix() {
        // +CPIN and +CEREG queries must arrive as data, not URCs.
        assert_eq!(classify("+CPIN: READY"), LineKind::Data);
        assert_eq!(classify("+CEREG: 1,5"), LineKind::Data);
    }

    #[test]
    fn cme_code_parses() {
        assert_eq!(cme_code("+CME ERROR: 4"), 4);
        assert_eq!(cme_code("+CME ERROR: 103"), 103);
        assert_eq!(cme_code("ERROR"), 0);
        assert_eq!(cme_code("+CME ERROR: jibberish"), 0);
    }

    #[test]
    fn field_splits_and_strips_quotes() {
        assert_eq!(field("+COPS: 0,2,\"Telkomsel\",7", 2).as_str(), "Telkomsel");
        assert_eq!(field("+COPS: 0,2,\"Telkomsel\",7", 0).as_str(), "0");
        assert_eq!(field("+CGMR: LE20B05", 0).as_str(), "LE20B05");
        assert!(field("+CGMR: LE20B05", 5).is_empty(), "out of range");
        assert!(field("bare line", 0).is_empty(), "no colon");
    }

    #[test]
    fn find_takes_first_prefix_match() {
        let v: HVec<&str, 4> = HVec::from_iter(["+CGPS: 0", "+CGPS: 0,1"]);
        assert_eq!(find(v.iter().copied(), "+CGPS"), Some("+CGPS: 0"));
        assert_eq!(find(["a", "b"], "+X"), None);
    }

    #[test]
    fn bare_value_extraction() {
        // With ATE0, AT+CGMM replies bare with leading blank lines.
        let mut lb = LineBuffer::new();
        let mut out = lines();
        lb.feed(b"\n\n\n\n\nSIM7600CE-T\n\n\n\n\nOK\n", &mut out);
        let got: HVec<&str, MAX_LINES> = HVec::from_iter(out[..8].iter().map(|s| s.as_str()));
        assert_eq!(first_non_empty(got.iter().copied()), Some("SIM7600CE-T"));
    }

    #[test]
    fn number_validation() {
        assert!(valid_number("+6281234567890").is_some());
        assert_eq!(valid_number("+62 812-3456-7890").unwrap().as_str(), "+6281234567890");
        assert!(valid_number("0812345678").is_some());
        assert!(valid_number("bad number!").is_none(), "injection rejected");
        assert!(valid_number("12").is_none(), "too short");
        assert!(valid_number("+62123456789012345").is_none(), "too long");
        assert!(valid_number("6281x\r\nAT+CMGS=\"x\"").is_none(), "CR injection");
    }

    #[test]
    fn apn_host_body_validation() {
        assert!(valid_apn("internet"));
        assert!(valid_apn("telkomsel-data_1-x"));
        assert!(!valid_apn("bad apn!"));
        assert!(!valid_apn(""));
        assert!(!valid_apn(&"a".repeat(65)));

        assert!(valid_host("example.com"));
        assert!(valid_host("api.example-co.io"));
        assert!(!valid_host("exa mple.com"));
        assert!(!valid_host(&"a".repeat(256)));

        assert!(valid_body("M 106.845,-6.21 s4 a42s tick")); // ASCII only
        assert!(!valid_body(""));
        assert!(!valid_body("body\r\nAT+CMGS=\"x\""), "control chars rejected");
        assert!(!valid_body(&"x".repeat(161)));
    }
}
