// Mock Arduino core for the host simulator (tools/sim). Only what the sketch uses.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <algorithm>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3

uint32_t millis();
void delay(uint32_t ms);
void pinMode(int pin, int mode);
int digitalRead(int pin);
void digitalWrite(int pin, int v);

template <class A, class B> auto max(A a, B b) -> decltype(a + b) { return a > b ? a : b; }
template <class A, class B> auto min(A a, B b) -> decltype(a + b) { return a < b ? a : b; }
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

class SimSerial {
 public:
  void begin(unsigned long) {}
  explicit operator bool() const;
  int available();
  int read();
  int availableForWrite();
  void flush() {}
  size_t write(uint8_t b);
  size_t write(const uint8_t *p, size_t n);
  size_t print(const char *s);
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(unsigned char v) { return printnum("%u", (unsigned)v); }
  size_t print(int v) { return printnum("%d", v); }
  size_t print(unsigned int v) { return printnum("%u", v); }
  size_t print(long v) { return printnum("%ld", v); }
  size_t print(unsigned long v) { return printnum("%lu", v); }
  size_t print(double v, int digits = 2);
  template <class T> size_t println(T v) { size_t n = print(v); return n + print("\r\n"); }
  size_t println(double v, int d) { size_t n = print(v, d); return n + print("\r\n"); }
  size_t println() { return print("\r\n"); }
 private:
  template <class T> size_t printnum(const char *f, T v) { char b[32]; snprintf(b, sizeof b, f, v); return print(b); }
};
extern SimSerial Serial;
