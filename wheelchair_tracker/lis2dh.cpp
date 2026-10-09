#include "lis2dh.h"

void Lis2dh::_w(uint8_t reg, uint8_t val) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->write(val);
  _wire->endTransmission();
}

uint8_t Lis2dh::_r(uint8_t reg) {
  // Guard: probe() is the natural first thing a caller tries, and before
  // begin() there is no bus. Return 0 rather than dereference a null wire.
  if (_wire == nullptr) return 0;
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) return 0;
  if (_wire->requestFrom((int)_addr, (int)1) != 1) return 0;
  return (uint8_t)_wire->read();
}

bool Lis2dh::probe() {
  uint8_t who = _r(LIS_WHO_AM_I);
  return who == 0x32 || who == 0x33 || who == 0x2A;
}

const char *Lis2dh::whoAmIToName(uint8_t v) {
  switch (v) {
    case 0x32: return "LIS2DH12";
    case 0x33: return "LIS3DH-family clone";
    case 0x2A: return "LIS2DU-family clone";
    default: return "unknown";
  }
}

bool Lis2dh::begin(TwoWire &wire, uint8_t addr) {
  _wire = &wire;
  _addr = addr;
  if (!probe()) return false;
  return configure();
}

bool Lis2dh::configure(uint8_t threshold, uint8_t duration) {
  if (!_wire || !probe()) return false;
  _w(LIS_CTRL_REG1, LIS_CTRL1_LP_10HZ);
  _w(LIS_CTRL_REG4, LIS_CTRL4_2G);
  _w(LIS_CTRL_REG2, 0x00);              // no high-pass filter
  _w(LIS_INT1_CFG, LIS_INT1_ANY_AXIS);
  _w(LIS_INT1_THS, threshold);
  _w(LIS_INT1_DUR, duration);
  _w(LIS_CTRL_REG5, 0x00);              // INT1 routed to the INT1 pad
  return true;
}

bool Lis2dh::dataReady() {
  return (_r(LIS_STATUS_REG) & 0x08) != 0;
}

bool Lis2dh::accel(int16_t &x, int16_t &y, int16_t &z) {
  uint32_t deadline = millis() + 200;
  while (!dataReady()) {
    if ((int32_t)(millis() - deadline) >= 0) return false;
  }

  _wire->beginTransmission(_addr);
  _wire->write(LIS_OUT_X_L);
  if (_wire->endTransmission(false) != 0) return false;
  if (_wire->requestFrom((int)_addr, (int)6) != 6) return false;

  uint8_t b[6];
  for (uint8_t i = 0; i < 6; i++) b[i] = (uint8_t)_wire->read();

  // 12-bit two's complement, left-justified in each 16-bit word; 1 mg/LSB in the
  // +-2 g range. Sign extension must come from bit 11 of the 12-bit value, not
  // from the 16-bit word: shifting -1000 mg right by 4 yields 3096, not -1000,
  // because the device emits 0x418 << 4 rather than a sign-extended 16-bit word.
  int16_t raw[3];
  for (uint8_t i = 0; i < 3; i++) {
    uint16_t w = (uint16_t)b[i * 2] | ((uint16_t)b[i * 2 + 1] << 8);
    int16_t v = (int16_t)(w >> 4);
    if (v & 0x0800) v |= (int16_t)0xF000;     // extend bit 11 downward
    raw[i] = v;
  }
  x = raw[0];
  y = raw[1];
  z = raw[2];
  return true;
}

int32_t Lis2dh::magnitude() {
  int16_t x, y, z;
  if (!accel(x, y, z)) return -1;
  uint32_t m = (uint32_t)((int32_t)x * x + (int32_t)y * y + (int32_t)z * z);
  // Integer square root, so this costs no flash and no float.
  uint32_t root = 0, bit = 1UL << 30;
  while (bit > m) bit >>= 2;
  while (bit) {
    if (m >= root + bit) {
      m -= root + bit;
      root = (root >> 1) + bit;
    } else {
      root >>= 1;
    }
    bit >>= 2;
  }
  return (int32_t)root;
}

bool Lis2dh::motionInterrupt() {
  return (_r(LIS_INT1_SRC) & 0x01) != 0;
}

void Lis2dh::clearInterrupt() {
  _r(LIS_INT1_SRC);                      // read has the clear side effect
}

bool Lis2dh::isMoving(uint16_t idleThresholdMg, uint8_t samples, uint16_t timeoutMs) {
  uint8_t hits = 0;
  uint32_t deadline = millis() + timeoutMs;
  while (hits < samples && (int32_t)(millis() - deadline) < 0) {
    delay(1);                    // yield while the sensor converts; a busy
                                  // spin here also starves the yield the
                                  // deadline logic depends on
    int32_t mag = magnitude();
    if (mag < 0) break;                 // no conversion ready
    long delta = (long)mag - 1000;
    if (delta < 0) delta = -delta;
    if (delta > (long)idleThresholdMg) {
      hits++;
    } else {
      hits = 0;                          // must be consecutive
    }
  }
  return hits >= samples;
}