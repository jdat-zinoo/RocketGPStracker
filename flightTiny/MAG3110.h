#pragma once
#include <stdint.h>

//template<uint8_t sa1 = 0>
class MAG3110 {
public:
  bool begin(void);

  void mode(uint8_t opMode);
  void configure1(uint8_t ctrl1);
  void configure2(uint8_t ctrl2);

  bool readMagX(int16_t &value);
  bool readMagY(int16_t &value);
  bool readMagZ(int16_t &value);
  bool readMag(int16_t &xvalue, int16_t &yvalue, int16_t &zvalue);
  uint8_t readMagT(void);

private:
  bool readByte(uint8_t addr, uint8_t *value);
  bool writeByte(uint8_t addr, uint8_t value);
  bool readWord(uint8_t addr, uint16_t *value);

  uint8_t   slaveAddress;  // 00111x0, where x = SA1 pin state
};
