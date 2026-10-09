/**
 * lis2dh.h - LIS2DH / LIS2DH12 accelerometer over I2C.
 *
 * Purpose here is motion gating, not measurement. The chair's state machine
 * needs to know "is this moving?" so it can decide whether to pay for a GNSS
 * acquisition, and a threshold interrupt is the right tool for a yes/no
 * question: the MCU stays asleep and the sensor decides when to wake it.
 *
 * The device is never placed in high-performance mode. Low-power mode at 10 Hz
 * is ~3 uA and there is no reason to spend more on a boolean.
 *
 * I2C address is 0x19 with SDO/SA0 high (usual breakout wiring), 0x18 with it
 * pulled low.
 */

#ifndef LIS2DH_H
#define LIS2DH_H

#include <Arduino.h>
#include <Wire.h>

// Registers, LIS2DH12 datasheet.
static const uint8_t LIS_WHO_AM_I = 0x0F;
static const uint8_t LIS_CTRL_REG1 = 0x10;
static const uint8_t LIS_CTRL_REG2 = 0x11;
static const uint8_t LIS_CTRL_REG4 = 0x1F;
static const uint8_t LIS_CTRL_REG5 = 0x20;
static const uint8_t LIS_STATUS_REG = 0x1E;
static const uint8_t LIS_INT1_CFG = 0x09;
static const uint8_t LIS_INT1_SRC = 0x0D;
static const uint8_t LIS_INT1_THS = 0x07;
static const uint8_t LIS_INT1_DUR = 0x08;
static const uint8_t LIS_OUT_X_L = 0x28;

// CTRL_REG1: LPen=1 low-power mode, BDU=1 block update, XYZ_EN=111, ODR=0010
// (10 Hz - the lowest rate that still gates reliably).
static const uint8_t LIS_CTRL1_LP_10HZ = 0x2A;

// CTRL_REG4: +-2 g so a chair bump is in range without clipping.
static const uint8_t LIS_CTRL4_2G = 0x08;

// INT1_CFG: latch on any of the three axes (bit7 AND-combined).
static const uint8_t LIS_INT1_ANY_AXIS = 0xFD;

class Lis2dh {
public:
  bool begin(TwoWire &wire, uint8_t addr = 0x19);

  // Low-power mode, 10 Hz, threshold interrupt on INT1.
  //   threshold: raw magnitude in 1/16 g, so 0x10 = 1 g. Start at 0x08 (0.5 g)
  //              and tune against the real chair - below ~0x04, floor
  //              vibration false-triggers constantly.
  //   duration:  consecutive samples above threshold before firing; 0 = 1.
  bool configure(uint8_t threshold = 0x08, uint8_t duration = 0);

  // True if WHO_AM_I matches a known LIS2DH family value.
  bool probe();

  bool dataReady();
  bool accel(int16_t &x, int16_t &y, int16_t &z);   // mg, false on timeout
  int32_t magnitude();                              // mg, ~1000 when still

  // Latched threshold interrupt. Reading INT1_SRC clears it (ST side effect).
  bool motionInterrupt();
  void clearInterrupt();

  // Polling fallback / cross-check, so a missed interrupt does not silently
  // read as "parked". Orientation-independent: gravity fixes the vector length
  // at 1 g regardless of how the chair is tilted.
  bool isMoving(uint16_t idleThresholdMg = 200, uint8_t samples = 3,
                uint16_t timeoutMs = 300);

  static const char *whoAmIToName(uint8_t v);

private:
  void _w(uint8_t reg, uint8_t val);
  uint8_t _r(uint8_t reg);

  TwoWire *_wire = nullptr;
  uint8_t _addr = 0x19;
};

#endif  // LIS2DH_H