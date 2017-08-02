#pragma once
//#include <stdint.h>

//#define byte uint8_t
#include <Arduino.h>

//#include <stdint.h>
//#include <inttypes.h>

//typedef uint8_t byte;
//typedef uint16_t word;

class I2C {
public:
  I2C();

  byte write(byte address, const char *buffer, byte bufSize);
  byte read(byte address, char *buffer, byte bufSize);

private:
  uint16_t rxTimeout;
};
