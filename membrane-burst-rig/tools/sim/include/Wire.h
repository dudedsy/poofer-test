// Mock Wire for the host simulator: routes transactions to emulated I2C devices.
#pragma once
#include <stdint.h>
#include <stddef.h>
class SimWire {
 public:
  bool setSDA(int) { return true; }
  bool setSCL(int) { return true; }
  void begin() {}
  void setClock(uint32_t) {}
  void setTimeout(int, bool) {}
  void beginTransmission(int addr);
  size_t write(uint8_t b);
  size_t write(const uint8_t *p, size_t n);
  uint8_t endTransmission(bool stop = true);
  size_t requestFrom(int addr, size_t n, bool stop = true);
  int read();
  int available();
 private:
  int addr_ = -1;
  uint8_t tx_[1024];
  size_t ntx_ = 0;
  uint8_t rx_[1024];
  size_t nrx_ = 0, rpos_ = 0;
};
extern SimWire Wire;
