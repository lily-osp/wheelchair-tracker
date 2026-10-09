#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include "String.h"

// Host stand-ins for the Arduino Print/Stream classes.
//
// print()/println() must route through write(), exactly as the real core does.
// A no-op print() would silently swallow everything the firmware sends, which
// makes "did we emit the right bytes?" assertions pass vacuously.

class Print {
public:
  virtual ~Print() {}
  virtual size_t write(uint8_t) { return 1; }

  size_t print(const char *s) {
    size_t n = 0;
    while (s && *s) n += write((uint8_t)*s++);
    return n;
  }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(const String &s) { return print(s.c_str()); }
  size_t print(int v) { char t[16]; snprintf(t, sizeof t, "%d", v); return print(t); }
  size_t print(unsigned v) { char t[16]; snprintf(t, sizeof t, "%u", v); return print(t); }
  size_t print(long v) { char t[24]; snprintf(t, sizeof t, "%ld", v); return print(t); }

  size_t println() { return print("\r\n"); }
  template <typename T> size_t println(const T &v) { return print(v) + println(); }
  int printf(const char *, ...) { return 0; }
};

class Stream : public Print {
public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() { return -1; }

  size_t readBytes(uint8_t *buf, size_t n) {
    size_t i = 0;
    while (i < n) {
      int c = read();
      if (c < 0) break;
      buf[i++] = (uint8_t)c;
    }
    return i;
  }

  void setTimeout(uint32_t) {}
};