//! GNSS fix parsing from AT-port position queries.
//!
//! The module has no second UART wired, so NMEA is not an option (issue #6).
//! Position comes as a single AT reply line. SIM7600 firmware varies here:
//! `AT+CGNSSINFO` includes satellite counts on some builds, `AT+CGPSINFO` on
//! others and errors entirely on a few (issue #7: the exact capability set is
//! unverified on this module, so the driver tries CGNSSINFO and falls back).
//!
//! Format (CGNSSINFO):
//!   +CGNSSINFO: [lat],[N/S],[lon],[E/W],[ddmmyy],[hhmmss.s],[alt],[speed],
//!               [course],[PDOP],[HDOP],[VDOP],[used],[in view]
//! Format (CGPSINFO):
//!   +CGPSINFO: [lat],[N/S],[lon],[E/W],[ddmmyy],[hhmmss.s],[alt],[speed],[course]

use heapless::String;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Fix {
    /// Degrees x 1e7.
    pub lat_e7: i32,
    pub lon_e7: i32,
    /// Satellites in use, 0 when the firmware does not report a count.
    pub sats: u8,
    /// Unix time (s) of the fix, 0 when no date/time in the reply.
    pub unix_time: u32,
}

impl Fix {
    pub fn valid(&self) -> bool {
        self.lat_e7 != 0 || self.lon_e7 != 0
    }
}

/// Parses a position reply line into a fix.
///
/// ⚠️ Satellite counts: CGNSSINFO reports "in use/in view" at indexes that
/// shift between firmware builds, and parsing a guessed count into a tracking
/// report is worse than omitting it (issue #7). `sats` is always 0 here; the
/// report body omits the field rather than lying.
pub fn parse_position_line(line: &str) -> Option<Fix> {
    if !(line.starts_with("+CGNSSINFO") || line.starts_with("+CGPSINFO")) {
        return None;
    }

    // Every field may be empty (the modem emits commas even for no data).
    // Firmware builds disagree on the leading fields (some emit
    // lat,N/S,lon,E/W,... directly; LE20B05SIM7600M22 prefixes
    // "status,sats_used,sats_view,fix_mode,"), so locate the hemisphere
    // markers instead of assuming fixed indices.
    const N: usize = 24;
    let mut fields: [String<32>; N] = Default::default();
    let Some(colon) = line.find(':') else {
        return None;
    };
    let body = line[colon + 1..].trim_start();
    let mut nfields = 0usize;
    // Split on commas, keeping empty fields. Stop at N.
    let mut pos = 0usize;
    while nfields < N {
        let rest = &body[pos..];
        if pos >= body.len() && nfields > 0 && !body.ends_with(',') {
            // no more fields
            // but if body ended with comma, we already added empty? handle below
            break;
        }
        // find next comma
        let (field, consumed) = match rest.find(',') {
            Some(rel) => (&rest[..rel], rel + 1),
            None => (rest, rest.len()),
        };
        fields[nfields].push_str(field.trim()).ok();
        nfields += 1;
        if consumed == rest.len() && !rest.contains(',') {
            // last field, no trailing comma
            break;
        }
        pos += consumed;
        if pos >= body.len() {
            // if original ended with comma, add one empty trailing field
            if body.ends_with(',') && nfields < N {
                fields[nfields].clear();
                nfields += 1;
            }
            break;
        }
    }
    if nfields == 0 {
        return None;
    }
    let all_empty = (0..nfields).all(|k| fields[k].is_empty());
    if all_empty {
        return None;
    }

    // Find first N/S hemisphere marker.
    let mut lat_idx: Option<usize> = None;
    let mut lon_idx: Option<usize> = None;
    for k in 0..nfields {
        match fields[k].as_str() {
            "N" | "S" => {
                if lat_idx.is_none() {
                    lat_idx = Some(k);
                }
            }
            "E" | "W" => {
                if lon_idx.is_none() && lat_idx.is_some() {
                    lon_idx = Some(k);
                }
            }
            _ => {}
        }
    }
    let li = lat_idx?;
    let oi = lon_idx?;
    if li == 0 || oi <= li + 1 {
        return None;
    }
    let lat_e7 = parse_hemisphere(fields[li - 1].as_str(), fields[li].as_str())?;
    let lon_e7 = parse_hemisphere(fields[oi - 1].as_str(), fields[oi].as_str())?;

    // Date/time follow the E/W marker.
    let date = if oi + 1 < nfields { fields[oi + 1].as_str() } else { "" };
    let time = if oi + 2 < nfields { fields[oi + 2].as_str() } else { "" };
    let unix_time = parse_unix_time(date, time).unwrap_or(0);

    Some(Fix {
        lat_e7,
        lon_e7,
        sats: 0,
        unix_time,
    })
}

/// "0653.264" x N/S -> degrees x 1e7 (65.544 N -> 655440000).
fn parse_hemisphere(value: &str, hemi: &str) -> Option<i32> {
    if value.is_empty() || hemi.is_empty() {
        return None;
    }
    let v: f64 = value.parse().ok()?;
    // "dddmm.mmm": whole degrees before the first two minute digits. Cast
    // instead of `f64::trunc` - the float math intrinsics are not in core.
    let deg = ((v / 100.0) as i64) as f64;
    let min = v - deg * 100.0;
    let d = deg + min / 60.0;
    let e7 = (d * 1e7) as i32;
    Some(match hemi {
        "S" | "W" => -e7,
        _ => e7,
    })
}

/// "ddmmyy" + "hhmmss.s" -> unix time, assuming UTC (GNSS is UTC).
fn parse_unix_time(date: &str, time: &str) -> Option<u32> {
    if date.len() < 6 {
        return None;
    }
    let d: u32 = date[0..2].parse().ok()?;
    let mo: u32 = date[2..4].parse().ok()?;
    let y: u32 = 2000 + date[4..6].parse::<u32>().ok()?;
    let h: u32 = time.get(0..2).and_then(|s| s.parse().ok()).unwrap_or(0);
    let mi: u32 = time.get(2..4).and_then(|s| s.parse().ok()).unwrap_or(0);
    let s: u32 = time.get(4..6).and_then(|s| s.parse().ok()).unwrap_or(0);

    let leap_years = (y - 1972) / 4 + 1;
    const CUM: [u32; 12] = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334];
    let mut days = (y - 1970) * 365 + leap_years + CUM[(mo - 1) as usize] + d - 1;
    if mo > 2 && is_leap(y) {
        days += 1;
    }
    Some(((days * 24 + h) * 60 + mi) * 60 + s)
}

fn is_leap(y: u32) -> bool {
    (y % 4 == 0 && y % 100 != 0) || y % 400 == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_cgpsinfo_fix() {
        // 6 deg 33.264' S, 106 deg 50.0' E
        let line = "+CGPSINFO: 0633.264,S,10650.000,E,291025,073055.0,32.0,0.0,180.0";
        let f = parse_position_line(line).unwrap();
        assert!(f.valid());
        assert!((f.lat_e7 - -65_544_000).abs() <= 1, "got {}", f.lat_e7);
        assert!((f.lon_e7 - 1_068_333_333).abs() <= 1, "got {}", f.lon_e7);
        assert_eq!(f.unix_time, 1_761_723_055);
        assert_eq!(f.sats, 0, "honest omission over a guessed count");
    }

    #[test]
    fn parses_cgnssinfo_fix() {
        let line = "+CGNSSINFO: 0633.264,S,10650.000,E,291025,073055.0,32.0,0.0,180.0,,,,,9,12,3";
        let f = parse_position_line(line).unwrap();
        assert!(f.valid());
        assert_eq!(f.sats, 0);
    }

    #[test]
    fn parses_cgnssinfo_fix_le20() {
        // Real capture from SIM7600CE-T firmware LE20B05SIM7600M22.
        let line = "+CGNSSINFO: 2,04,02,01,0613.042428,S,10647.601116,E,311026,161625.0,23.5,0.0,,1.5,1.2,1.0";
        let f = parse_position_line(line).unwrap();
        assert!(f.valid());
        // ~6.13 S, 106.47 E
        assert_eq!(f.lat_e7, -62_173_738, "got {}", f.lat_e7);
        assert_eq!(f.lon_e7, 1_067_933_519, "got {}", f.lon_e7);
    }

    #[test]
    fn empty_reply_is_not_a_fix() {
        assert!(parse_position_line("+CGPSINFO: ,,,,,,,,,,,,,,").is_none());
        assert!(parse_position_line("+CGPSINFO: ").is_none());
    }

    #[test]
    fn other_lines_are_ignored() {
        assert!(parse_position_line("OK").is_none());
        assert!(parse_position_line("+CGPS: 0,1").is_none());
    }

    #[test]
    fn no_date_means_zero_unix_time() {
        let line = "+CGPSINFO: 0633.264,S,10650.000,E,,,,,,,,,";
        let f = parse_position_line(line).unwrap();
        assert!(f.valid());
        assert_eq!(f.unix_time, 0);
    }

    #[test]
    fn north_east_positive() {
        let line = "+CGPSINFO: 4001.000,N,11630.000,E,291025,000000.0,0,0,0";
        let f = parse_position_line(line).unwrap();
        assert!(f.lat_e7 > 0);
        assert!(f.lon_e7 > 0);
    }
}
