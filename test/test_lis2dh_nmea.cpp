/**
 * Host-side tests for the Arduino C++ firmware.
 *
 * Compiles lis2dh.cpp and Nmea.cpp against a fake I2C and a scripted clock, so
 * the parts that can be verified without hardware are verified. The AT
 * transport has its own harness in test_sim7600_at.cpp.
 *
 * NOT part of the firmware. Build:
 *   sh test/run.sh
 */

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <vector>
#include <string>

#include "Wire.h"     // host TwoWire
#include "lis2dh.h"   // register constants, needed by the fake below

// The shim's clock is per-translation-unit, so advance it directly here.
namespace shim_clock { extern void advance(uint32_t); }
#define ADVANCE_MS(ms) shim_clock::advance(ms)


static int g_pass = 0, g_fail = 0;
static void check(const char *label, bool ok, const std::string &got = "") {
  printf("  %s  %-40s %s\n", ok ? "PASS" : "FAIL", label, got.c_str());
  fflush(stdout);
  ok ? g_pass++ : g_fail++;
}

// Fake I2C: an LIS2DH-shaped register file, delivered through the same
// virtual TwoWire interface the firmware uses.
class FakeWire : public TwoWire {
public:
  uint8_t who = 0x32;
  uint8_t regs[256] = {0};
  int16_t x = 1000, y = 0, z = 0;
  bool ready = true;
  int irqLatched = 0;
  std::vector<std::string> writes;

  void reset() {
    memset(regs, 0, sizeof(regs));
    regs[LIS_STATUS_REG] = ready ? 0x08 : 0x00;
  }

  void beginTransmission(uint8_t) override { pending.clear(); }
  size_t write(uint8_t v) override { pending.push_back(v); return 1; }
  uint8_t endTransmission(bool stopAll = true) override {
    (void)stopAll;
    if (pending.empty()) return 1;
    uint8_t reg = pending[0];
    if (pending.size() > 1) {
      regs[reg] = pending[1];
      char buf[8];
      snprintf(buf, sizeof(buf), "%02x=%02x", reg, pending[1]);
      writes.push_back(buf);
    }
    lastReg = reg;
    pending.clear();
    return 0;
  }
  int requestFrom(uint8_t, int n) override { requestN = n; return n; }
  int read() override {
    if (lastReg == LIS_WHO_AM_I) return who;
    if (lastReg == LIS_STATUS_REG) return regs[LIS_STATUS_REG];
    if (lastReg == LIS_INT1_SRC) return irqLatched;
    if (lastReg == LIS_OUT_X_L) {
      // requestFrom(6) delivers six consecutive bytes: low/high word per axis.
      // 12-bit left-justified, as the device outputs it.
      int16_t vals[3] = {x, y, z};
      int word = streamIdx / 2;          // which axis
      int half = streamIdx % 2;          // low or high byte
      streamIdx++;
      uint16_t w = (uint16_t)((uint16_t)vals[word & 3] << 4);
      return half == 0 ? (w & 0xFF) : ((w >> 8) & 0xFF);
    }
    return regs[lastReg];
  }

  uint8_t lastReg = 0;
  int requestN = 0;
  int streamIdx = 0;
  std::vector<uint8_t> pending;
};

// ---- NMEA tests ----------------------------------------------------------

#include "Nmea.h"

static std::string withCks(const char *body) {
  uint8_t sum = 0;
  for (const char *p = body; *p; p++) sum ^= (uint8_t)*p;
  char buf[256];
  snprintf(buf, sizeof(buf), "$%s*%02X", body, sum);
  return buf;
}

// Feed one sentence terminated by CRLF, as the receiver emits it. The newline
// is what completes a line - without it nothing is ever applied.
static bool feedLine(NmeaFix &f, const char *line) {
  bool applied = false;
  for (const char *p = line; *p; p++) {
    if (nmeaFeed(f, *p)) applied = true;
  }
  if (nmeaFeed(f, '\r')) applied = true;
  if (nmeaFeed(f, '\n')) applied = true;
  return applied;
}

int main() {
  printf("== NMEA parser ==\n");
  {
    NmeaFix f;
    char got[160];
    bool applied = feedLine(f, withCks(
        "GNGGA,083055.00,0314.10336,S,10311.44321,E,1,08,0.9,545.4,M,8.0,M,,").c_str());
    snprintf(got, sizeof(got), "%.5f %.5f sats=%u q=%u alt=%.1f",
             f.lat, f.lon, f.sats, f.fixQuality, f.altM);
    // "0314.10336" is 03 deg 14.10336 min, south -> -3.2350560. Reading the
    // degree field as 3 digits would give -31.235056, a 28-degree error.
    check("GNGGA lat/lon (0314.10336 -> -3.235056, not -31)",
          applied && std::fabs(f.lat + 3.2350560) < 1e-5 &&
          std::fabs(f.lon - 103.1907203) < 1e-5 && f.sats == 8 &&
          f.fixQuality == 1, got);
  }
  {
    NmeaFix f;
    feedLine(f, withCks(
        "GNGGA,083055.00,0314.10336,S,10311.44321,E,1,08,0.9,545.4,M,8.0,M,,").c_str());
    check("valid() true after GGA", f.valid());
  }
  {
    NmeaFix f;
    feedLine(f, withCks(
        "GNRMC,083055.00,A,0314.10336,S,10311.44321,E,0.511,84.4,091026,,").c_str());
    char got[80];
    snprintf(got, sizeof(got), "%.1f km/h course %.1f", f.speedKmh, f.courseDeg);
    check("GNRMC knots -> km/h", std::fabs(f.speedKmh - 0.9464f) < 0.01f,
          got);
  }
  {
    NmeaFix f;
    feedLine(f, withCks(
        "GNGGA,000000.00,3352.00000,S,15112.00000,W,1,05,1.0,10.0,M,,").c_str());
    char got[80];
    snprintf(got, sizeof(got), "%.4f %.4f", f.lat, f.lon);
    check("S/W hemispheres are negative",
          f.lat < -33.86 && f.lon < -151.19, got);
  }
  {
    // Bad checksum must be rejected outright, not parsed.
    NmeaFix f;
    std::string bad = withCks(
        "GNGGA,083055.00,0314.10336,S,10311.44321,E,1,08,0.9,545.4,M,8.0,M,,");
    bad[bad.size() - 2] = '0';
    bad[bad.size() - 1] = '0';
    check("bad checksum rejected", !feedLine(f, bad.c_str()) && !f.valid());
  }
  {
    NmeaFix f;
    check("garbage rejected", !feedLine(f, "not an nmea sentence at all"));
  }
  {
    NmeaFix f;
    check("truncated sentence rejected", !feedLine(f, "$GNGGA,08*FF"));
  }
  {
    NmeaFix f;
    feedLine(f, withCks("GPGSV,3,1,11,01,55,213,44,02,17,308,41,12,07,344,39").c_str());
    check("GSV is valid syntax but applies nothing", !f.valid());
  }
  {
    // Byte-at-a-time across a line boundary.
    NmeaFix f;
    std::string s = withCks("GNGGA,000000.00,0614.00000,N,10311.00000,E,1,09,0.8,12.0,M,,");
    s += "\r\n";
    bool applied = false;
    for (char c : s) applied |= nmeaFeed(f, c);
    check("CR ignored, sentence still applies", applied && f.valid());
  }
  {
    // A second fix on the same Fix must not accumulate garbage.
    NmeaFix f;
    feedLine(f, withCks("GNGGA,000000.00,0614.00000,N,10311.00000,E,1,09,0.8,12.0,M,,").c_str());
    feedLine(f, withCks("GNGGA,000000.00,0614.00000,N,10311.00000,E,1,09,0.8,99.0,M,,").c_str());
    char got[64];
    snprintf(got, sizeof(got), "alt=%.1f", f.altM);
    check("later sentence overwrites", std::fabs(f.altM - 99.0f) < 0.01f, got);
  }

  printf("\n== LIS2DH ==\n");
  {
    FakeWire w;
    w.reset();
    Lis2dh imu;
    bool ok = imu.begin(w, 0x19);
    check("begin() with 0x32 WHO_AM_I", ok);
    check("low-power 10Hz written to CTRL_REG1",
          w.regs[LIS_CTRL_REG1] == LIS_CTRL1_LP_10HZ);
    check("+-2g written to CTRL_REG4", w.regs[LIS_CTRL_REG4] == LIS_CTRL4_2G);
    check("threshold written to INT1_THS", w.regs[LIS_INT1_THS] == 0x08);
  }
  {
    FakeWire w;
    w.who = 0x00;
    w.reset();
    Lis2dh imu;
    check("wrong WHO_AM_I rejected", !imu.probe());
  }
  {
    FakeWire w;
    w.reset();
    Lis2dh imu;
    imu.begin(w, 0x19);
    w.x = 1000; w.y = 0; w.z = 0;
    int16_t x, y, z;
    imu.accel(x, y, z);
    char got[64];
    snprintf(got, sizeof(got), "%d %d %d", x, y, z);
    check("accel decodes left-justified 12-bit", x == 1000 && y == 0 && z == 0, got);
  }
  {
    FakeWire w;
    w.reset();
    Lis2dh imu;
    imu.begin(w, 0x19);
    w.x = -1000; w.y = 500; w.z = 0;
    int16_t x, y, z;
    imu.accel(x, y, z);
    char got[64];
    snprintf(got, sizeof(got), "%d %d %d", x, y, z);
    check("negative values sign-extend", x == -1000 && y == 500, got);
  }
  {
    FakeWire w;
    w.reset();
    Lis2dh imu;
    imu.begin(w, 0x19);
    w.x = 1000; w.y = 0; w.z = 0;
    int32_t m = imu.magnitude();
    char got[32];
    snprintf(got, sizeof(got), "%ld", (long)m);
    check("magnitude of 1g is 1000", m == 1000, got);
  }
  {
    FakeWire w;
    w.reset();
    Lis2dh imu;
    imu.begin(w, 0x19);
    w.x = 1800; w.y = 1800; w.z = 0;
    int32_t m = imu.magnitude();
    check("magnitude of a 3g-slope vector > 1000", m > 2500);
  }
  {
    FakeWire w;
    w.reset();
    Lis2dh imu;
    imu.begin(w, 0x19);
    w.x = 1000; w.y = 0; w.z = 0;
    check("still -> not moving", !imu.isMoving(200, 2, 100));
  }
  {
    FakeWire w;
    w.reset();
    Lis2dh imu;
    imu.begin(w, 0x19);
    w.x = 2000; w.y = 2000; w.z = 2000;
    check("accelerating -> moving", imu.isMoving(200, 2, 100));
  }
  {
    FakeWire w;
    w.reset();
    Lis2dh imu;
    imu.begin(w, 0x19);
    w.irqLatched = 0x01;
    check("threshold interrupt latched", imu.motionInterrupt());
    w.irqLatched = 0x00;
    check("interrupt clear reads 0", !imu.motionInterrupt());
  }

  printf("\n%d/%d passed\n", g_pass, g_pass + g_fail);
  return g_fail ? 1 : 0;
}