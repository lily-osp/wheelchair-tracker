#pragma once
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <string>

// Mirrors the subset of Arduino String that Sim7600At.cpp uses. Backed by
// std::string on the host only; the firmware uses the real Arduino String.
class String {
public:
  String() {}
  String(const char *s) : _b(s ? s : "") {}
  String(const char *s, size_t n) { _b.assign(s, n); }
  String(int v) { char t[24]; snprintf(t, sizeof t, "%d", v); _b = t; }

  const char *c_str() const { return _b.c_str(); }
  size_t length() const { return _b.length(); }
  char operator[](size_t i) const { return _b[i]; }
  bool startsWith(const char *p) const { return _b.rfind(p, 0) == 0; }
  int indexOf(char c, int from = 0) const {
    size_t p = _b.find(c, from < 0 ? 0 : (size_t)from);
    return p == std::string::npos ? -1 : (int)p;
  }
  String substring(int a, int b = -1) const {
    if (a < 0 || a > (int)_b.length()) return String();
    if (b < 0 || b > (int)_b.length()) return String(_b.substr(a).c_str());
    return String(_b.substr(a, (size_t)(b - a)).c_str());
  }
  void replace(const char *from, const char *to) {
    size_t p = 0;
    while ((p = _b.find(from, p)) != std::string::npos) {
      _b.replace(p, strlen(from), to);
      p += strlen(to);
    }
  }
  bool operator==(const char *o) const { return _b == o; }
  bool operator==(const String &o) const { return _b == o._b; }
  bool operator!=(const char *o) const { return !(*this == o); }
  bool operator<(const String &o) const { return _b < o._b; }
  String &operator+=(char c) { _b += c; return *this; }
  String &operator+=(const char *s) { _b += s; return *this; }
  String &operator+=(const String &s) { _b += s._b; return *this; }

private:
  std::string _b;
};
