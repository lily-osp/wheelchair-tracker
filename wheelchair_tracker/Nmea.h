/**
 * Nmea.h - incremental NMEA sentence parser for the tracker.
 *
 * Streams bytes in, accumulates a line, verifies its XOR checksum, then folds
 * the useful fields into a NmeaFix. Written as a byte-at-a-time feeder because
 * the GNSS UART delivers in arbitrary chunk sizes.
 *
 * Two details that are easy to get wrong:
 *   - The tag carries a talker id on multi-constellation receivers ("GNGGA"),
 *     so match on the trailing sentence name only.
 *   - GGA/RMC coordinate fields are fixed-width ddmm.mmmm / dddmm.mmmm. The
 *     degree split is positional; counting digits before the decimal point
 *     gives 31 degrees for "03114.10336" when it should be 3.
 */

#ifndef NMEA_H
#define NMEA_H

#include <Arduino.h>

struct NmeaFix {
  double lat = 0.0;
  double lon = 0.0;
  float altM = 0.0f;
  float speedKmh = 0.0f;
  float courseDeg = 0.0f;
  uint8_t sats = 0;
  uint8_t fixQuality = 0;    // 0 none, 1 GPS, 2 DGPS
  uint8_t fixMode = 0;       // 1 = 3D, 2 = 2D
  char utc[16] = {0};
  bool latValid = false;
  bool lonValid = false;

  bool valid() const { return fixQuality > 0 && latValid && lonValid; }
};

// True if the sentence's trailing XOR checksum matches.
bool nmeaChecksumOk(const char *line, size_t len);

// Feed one byte. Returns true if it completed a sentence that was applied.
bool nmeaFeed(NmeaFix &fix, char c);

#endif  // NMEA_H