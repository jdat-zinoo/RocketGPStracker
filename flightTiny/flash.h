#pragma once
#include <stdint.h>

#include "Eeprom24C512.h"
#include "tiny.h"
//#include <Arduino.h>

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

typedef struct i2cLogEntry {
  uint8_t   stateLog;
  uint16_t  pressure;
  uint16_t  altitude;
  int  magX;
  int  magY;
  int  magZ;
  uint8_t   magT;
};

class flash {
public:
  uint8_t begin(void);
  i2cLogEntry entry;
  int pe=&entry;
  bool readByte(uint16_t addr, uint8_t &val);
  bool writeByte(uint16_t addr, uint8_t val);
  
  bool readRecord(uint16_t idx);
  bool writeRecord(uint16_t idx);
  //bool readWord(uint8_t addr, uint16_t *val);
private:
  //uint8_t   slaveAddress=0x50;  // 00111x0, where x = SA1 pin state
};
