#pragma once
#include <stdint.h>
//#include <Arduino.h>

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

//template<uint8_t sa1 = 0>
class flash {
public:
  uint8_t begin(void);

  bool readByte(uint16_t addr, uint8_t &val);
  bool writeByte(uint16_t addr, uint8_t val);
  //bool readWord(uint8_t addr, uint16_t *val);
private:
  uint8_t   slaveAddress;  // 00111x0, where x = SA1 pin state
};
