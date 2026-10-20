// Just enough Arduino for the sketch to run on a PC
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
inline uint32_t simUs = 0;
inline int pinVal[40];
inline int pwmVal[40];
inline uint32_t micros() { return simUs; }
inline uint32_t millis() { return simUs / 1000; }
inline void delay(uint32_t ms) { simUs += ms * 1000; }
inline void pinMode(int, int) {}
inline void digitalWrite(int p, int v) { pinVal[p] = v; }
inline int digitalRead(int p) { return p == 0 ? HIGH : pinVal[p]; }
inline void ledcAttach(int, uint32_t, uint8_t) {}
inline void ledcWrite(int p, int v) { pwmVal[p] = v; }
struct SerialStub {
  void begin(int) {}
  int available() { return 0; }
  int read() { return -1; }
  float parseFloat() { return 0; }
  template <class... A> void printf(const char *, A...) {}
  void println(const char * = "") {}
} inline Serial;
