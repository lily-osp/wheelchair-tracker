// Host Arduino shim. Lets the firmware's .cpp files compile and be unit-tested
// off-hardware. NOT part of the firmware - the board gets the real core.
#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <cstdio>
#include <cmath>
#include "Print.h"
#include "String.h"

// Time only advances when the driver yields. A frozen clock would make the
// deadline logic spin forever instead of timing out.
namespace shim_clock {
inline uint32_t &now() { static uint32_t t = 0; return t; }
inline void advance(uint32_t ms) { now() += ms; }
}
inline uint32_t millis() { return shim_clock::now(); }
inline void delay(uint32_t ms) { shim_clock::advance(ms); }
inline void delayMicroseconds(uint32_t us) { shim_clock::advance(us / 1000 + 1); }
