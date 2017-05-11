#include "MAG3110.h"
#include "TinyWireM.h"

//#include <Arduino.h>
#include "tiny.h"

#define regDR_STATUS 0x00

#define regOUT_X_MSB 0x01
#define regOUT_X_LSB 0x02
#define regOUT_Y_MSB 0x03
#define regOUT_Y_LSB 0x04
#define regOUT_Z_MSB 0x05
#define regOUT_Z_LSB 0x06

#define regWHO_AM_I  0x07
#define regSYSMOD 0x08

#define regOFF_X_MSB 0x09
#define regOFF_X_LSB 0x0a
#define regOFF_Y_MSB 0x0B
#define regOFF_Y_LSB 0x0C
#define regOFF_Z_MSB 0x0D
#define regOFF_Z_LSB 0x0E

#define regDIE_TEMP 0x0f

#define regCTRL_REG1 0x10
#define regCTRL_REG2 0x11


bool MAG3110::begin(void) {
  //slaveAddress = (SA1pin) ? 0b0011110 : 0b0011100;
  //dbg.begin();
  slaveAddress = 0x0E;
  // Try to read device ID
  uint8_t id;
  if (!readByte(regWHO_AM_I, &id)) {
    //dbg << "Begin: " id << endl;
    return false;
  }
  return id; // == 0xC4;
}

bool MAG3110::readByte(uint8_t addr, uint8_t *value) {
  uint8_t rc;
  TinyWireM.beginTransmission(slaveAddress);
  TinyWireM.send(addr);
  rc = TinyWireM.endTransmission();
  if (rc != 0) return false;

  rc = TinyWireM.requestFrom(slaveAddress, 1); // Request 1 byte from slave
  if (rc != 0) return false;

  if (TinyWireM.available() == 0) return false;
  *value = TinyWireM.receive();
  return true;
}

bool MAG3110::writeByte(uint8_t addr, uint8_t value) {
  uint8_t rc;
  TinyWireM.beginTransmission(slaveAddress);
  TinyWireM.send(addr);
  TinyWireM.send(value);
  rc = TinyWireM.endTransmission();
  if (rc != 0) return false;
  return true;
}

bool MAG3110::readWord(uint8_t addr, uint16_t *value) {
  uint8_t rc;
  TinyWireM.beginTransmission(slaveAddress);
  TinyWireM.send(addr );
  rc = TinyWireM.endTransmission();
  if (rc != 0) return false;

  rc = TinyWireM.requestFrom(slaveAddress, 2); // Request 1 byte from slave
  if (rc != 0) return false;

  if (TinyWireM.available() < 2) return false;

  uint8_t hi = TinyWireM.receive();
  uint8_t lo = TinyWireM.receive();
  //uint8_t hi = TinyWireM.receive();
  *value = (hi << 8) | lo;
  return true;
}

void MAG3110::configure1(uint8_t ctrl1) {
  uint8_t value = ctrl1;
  writeByte(regCTRL_REG1, value);
}

void MAG3110::configure2(uint8_t ctrl2) {
  uint8_t value = ctrl2;
  writeByte(regCTRL_REG2, value);
}

void MAG3110::mode(uint8_t opMode) {
  uint8_t value = opMode;
  writeByte(regSYSMOD, value);
}

bool MAG3110::readMagX(int16_t &value) {
  return readWord(regOUT_X_MSB, (uint16_t *)&value);
}

bool MAG3110::readMagY(int16_t &value) {
  return readWord(regOUT_Y_MSB, (uint16_t *)&value);
}

bool MAG3110::readMagZ(int16_t &value) {
  return readWord(regOUT_Z_MSB, (uint16_t *)&value);
}

bool MAG3110::readMag(int16_t &xvalue, int16_t &yvalue, int16_t &zvalue) {
  if (!readMagX(xvalue)) return false;
  if (!readMagY(yvalue)) return false;
  if (!readMagZ(zvalue)) return false;
  return true;
}
