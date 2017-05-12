#pragma once
#include <stdint.h>
//#include <Arduino.h>

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

struct i2cLogEntry {
  uint8_t   stateLog;
  uint16_t  pressure;
  uint16_t  magX;
  uint16_t  magY;
  uint16_t  magZ;
  uint8_t   magT;
};

class flash {
public:
  uint8_t begin(void);
  i2cLogEntry entry;

  bool readByte(uint16_t addr, uint8_t &val);
  bool writeByte(uint16_t addr, uint8_t val);
  //bool readWord(uint8_t addr, uint16_t *val);
private:
  uint8_t   slaveAddress;  // 00111x0, where x = SA1 pin state
};
