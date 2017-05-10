#include "flash.h"
#include "TinyWireM.h"

//#include <Arduino.h>
#include "tiny.hh"

bool flash::begin(void) {
  slaveAddress = 0x50;    //7-bit address 
  //slaveAddress = 0xA0;    //8-bit address
  
  // Try to read device ID
  uint8_t id;
  //if (!readByte(regWHO_AM_I, &id)) {
    //dbg << "Begin: " id << endl;
    //return false;
  //}
  return id; // == 0xC4;
}

bool flash::readByte(uint16_t addr, uint8_t *value) {
  uint8_t rc;
  TinyWireM.beginTransmission(slaveAddress);
  TinyWireM.send(highByte(addr));
  TinyWireM.send(lowByte(addr));
  rc = TinyWireM.endTransmission();
  if (rc != 0) return false;

  rc = TinyWireM.requestFrom(slaveAddress, 1); // Request 1 byte from slave
  if (rc != 0) return false;

  if (TinyWireM.available() == 0) return false;
  *value = TinyWireM.receive();
  return true;
}

bool flash::writeByte(uint16_t addr, uint8_t value) {
  uint8_t rc;
  TinyWireM.beginTransmission(slaveAddress);
  TinyWireM.send(highByte(addr));
  TinyWireM.send(lowByte(addr));
  TinyWireM.send(value);
  rc = TinyWireM.endTransmission();
  if (rc != 0) return false;
  return true;
}

/*
bool flash::readWord(uint8_t addr, uint16_t *value) {
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
*/
