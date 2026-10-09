#pragma once
#include <cstdint>
#include <cstddef>
#include "Print.h"

// Host stand-in for TwoWire, with virtual hooks so a test can supply a
// register-file fake. Virtual rather than the real driver's inline methods:
// the firmware only ever calls the virtual interface.
class TwoWire {
public:
  void begin(int sda = -1, int scl = -1, uint32_t freq = 0) { (void)sda;(void)scl;(void)freq; }
  void setClock(uint32_t) {}
  virtual void beginTransmission(uint8_t addr) { (void)addr; }
  virtual size_t write(uint8_t) { return 1; }
  virtual uint8_t endTransmission(bool sendStop = true) { (void)sendStop; return 0; }
  virtual uint8_t requestFrom(uint8_t addr, uint8_t count) { (void)addr;(void)count; return 0; }
  virtual int requestFrom(uint8_t addr, int count) { (void)addr;(void)count; return 0; }
  virtual int read() { return -1; }
};
