#include "Nmea.h"
#include <string.h>
#include <stdlib.h>

namespace {

// Line accumulator. 128 bytes covers the longest sentence in practice (GNGSV
// reaches ~90). Longer input is dropped rather than wrapped: a wrapped line
// could parse as a valid short sentence and produce a plausible wrong fix.
char g_line[128];
uint8_t g_len = 0;
bool g_overflow = false;

char g_fields[16][24];
uint8_t g_fieldCount = 0;

// NMEA ddmm.mmmm / dddmm.mmmm -> signed decimal degrees. Fixed widths, so the
// split is positional. Counting digits before the decimal point would read
// "03114.10336" as 31 degrees when it is 3.
double toDegrees(const char *raw, uint8_t degDigits) {
  if (raw == nullptr || *raw == '\0') return 0.0;
  double deg = 0.0;
  for (uint8_t i = 0; i < degDigits; i++) {
    if (raw[i] < '0' || raw[i] > '9') return 0.0;
    deg = deg * 10.0 + (raw[i] - '0');
  }
  return deg + atof(raw + degDigits) / 60.0;
}

void split(const char *body) {
  g_fieldCount = 0;
  const char *start = body;
  for (;;) {
    const char *comma = strchr(start, ',');
    size_t n = comma ? (size_t)(comma - start) : strlen(start);
    if (g_fieldCount < 16) {
      if (n > 23) n = 23;
      memcpy(g_fields[g_fieldCount], start, n);
      g_fields[g_fieldCount][n] = '\0';
      g_fieldCount++;
    }
    if (!comma) return;
    start = comma + 1;
  }
}

const char *field(uint8_t i) {
  return i < g_fieldCount ? g_fields[i] : "";
}

float toFloat(const char *s) {
  return (*s == '\0') ? 0.0f : (float)atof(s);
}

bool parseDegrees(const char *raw, const char *hemi, uint8_t degDigits,
                  bool isLon, NmeaFix &fix) {
  if (raw == nullptr || *raw == '\0' || hemi == nullptr || *hemi == '\0') {
    return false;
  }
  double v = toDegrees(raw, degDigits);
  if (v == 0.0 && (raw[0] < '0' || raw[0] > '9')) return false;
  if (hemi[0] == 'S' || hemi[0] == 'W') v = -v;
  if (isLon) {
    fix.lon = v;
    fix.lonValid = true;
  } else {
    fix.lat = v;
    fix.latValid = true;
  }
  return true;
}

// GGA: 0 utc, 1 lat, 2 NS, 3 lon, 4 EW, 5 quality, 6 sats, 7 hdop, 8 alt
// RMC: 0 utc, 1 status, 2 lat, 3 NS, 4 lon, 5 EW, 6 knots, 7 course, 8 date
// GSA: 2 sats used, 15 pdop
bool apply(const char *tag, NmeaFix &fix) {
  if (strcmp(tag, "GGA") == 0 && g_fieldCount >= 9) {
    if (field(0)[0] != '\0') {
      strncpy(fix.utc, field(0), sizeof(fix.utc) - 1);
    }
    parseDegrees(field(1), field(2), 2, false, fix);
    parseDegrees(field(3), field(4), 3, true, fix);
    if (field(5)[0] != '\0') fix.fixQuality = (uint8_t)atoi(field(5));
    if (field(6)[0] != '\0') fix.sats = (uint8_t)atoi(field(6));
    fix.altM = toFloat(field(8));
    return true;
  }
  if (strcmp(tag, "RMC") == 0 && g_fieldCount >= 8) {
    if (field(1)[0] == 'A') {              // status 'A' = valid
      parseDegrees(field(2), field(3), 2, false, fix);
      parseDegrees(field(4), field(5), 3, true, fix);
      fix.speedKmh = toFloat(field(6)) * 1.852f;   // knots -> km/h
      fix.courseDeg = toFloat(field(7));
      if (field(0)[0] != '\0') {
        strncpy(fix.utc, field(0), sizeof(fix.utc) - 1);
      }
      fix.fixQuality = 1;                  // an A-status fix is a fix
    }
    return true;
  }
  if (strcmp(tag, "GSA") == 0 && g_fieldCount >= 3) {
    if (field(2)[0] != '\0') fix.sats = (uint8_t)atoi(field(2));
    if (field(2)[0] != '\0') fix.fixMode = 2;       // no 3D flag parsed here
    return true;
  }
  // GSV and anything else: valid syntax, nothing to apply.
  return false;
}

}  // namespace

bool nmeaChecksumOk(const char *line, size_t len) {
  if (len < 4 || line[0] != '$') return false;
  const char *star = nullptr;
  for (size_t i = len; i > 0; i--) {
    if (line[i - 1] == '*') {
      star = line + i - 1;
      break;
    }
  }
  if (!star || star == line || star[1] == '\0') return false;

  uint8_t sum = 0;
  for (const char *p = line + 1; p < star; p++) sum ^= (uint8_t)*p;

  // Trailing two hex digits, HIGH nibble first (standard NMEA, *5E not *E5).
  auto hex = [](char c, uint8_t &out) -> bool {
    if (c >= '0' && c <= '9') { out = (uint8_t)(c - '0'); return true; }
    if (c >= 'A' && c <= 'F') { out = (uint8_t)(c - 'A' + 10); return true; }
    if (c >= 'a' && c <= 'f') { out = (uint8_t)(c - 'a' + 10); return true; }
    return false;
  };
  uint8_t hi, lo;
  if (!hex(star[1], hi)) return false;
  if (!hex(star[2], lo)) return false;
  return sum == (uint8_t)((hi << 4) | lo);
}

bool nmeaFeed(NmeaFix &fix, char c) {
  if (c != '\n') {
    if (c == '\r') return false;
    if (g_len < sizeof(g_line) - 1) g_line[g_len++] = c;
    else g_overflow = true;
    return false;
  }

  // End of line: validate, split, apply.
  if (g_overflow || g_len == 0) {
    g_len = 0;
    g_overflow = false;
    return false;
  }
  g_line[g_len] = '\0';

  bool applied = false;
  if (g_line[0] == '$' && nmeaChecksumOk(g_line, g_len)) {
    const char *star = strchr(g_line, '*');
    size_t bodyLen = star ? (size_t)(star - g_line - 1) : strlen(g_line);
    if (bodyLen >= 3 && bodyLen < 112) {
      char body[112];
      memcpy(body, g_line + 1, bodyLen);
      body[bodyLen] = '\0';

      // Take the tag off the front, then split only the data fields, so
      // g_fields[0] is always the first data field and the documented indices
      // ("GGA 1 = lat") stay true.
      char *comma = strchr(body, ',');
      if (comma) {
        *comma = '\0';
        size_t tl = strlen(body);
        if (tl >= 3) {
          char tag[4];
          memcpy(tag, body + tl - 3, 3);   // strip the talker id: "GNGGA"->"GGA"
          tag[3] = '\0';
          split(comma + 1);
          applied = apply(tag, fix);
        }
      }
    }
  }

  g_len = 0;
  g_overflow = false;
  return applied;
}