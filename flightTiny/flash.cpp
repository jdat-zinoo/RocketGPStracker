#include "flash.h"
#include "TinyWireM.h"

#include "tiny.h"

uint8_t flash::begin(void) {
  slaveAddress = 0x50;    //7-bit address 
  return slaveAddress; // == 0xC4;
}

bool dummyRead(uint8_t r,uint8_t *v){
  *v=r;
}

bool flash::readByte(uint16_t addr, uint8_t &val) {
  uint8_t rc;
  TinyWireM.beginTransmission(slaveAddress);
  TinyWireM.write(highByte(addr));
  TinyWireM.write(lowByte(addr));
  rc = TinyWireM.endTransmission(false);
  if (rc != 0) return false;
  rc = TinyWireM.requestFrom(slaveAddress, 1); // Request 1 byte from slave
  if (rc != 0) return false;
  if (TinyWireM.available() != 1) return false;
  val = TinyWireM.read();
  //dummyRead(rc,(uint8_t *)&val);
  //val=rc;
  return true;
}

bool flash::writeByte(uint16_t addr, uint8_t val) {
  uint8_t rc;
  TinyWireM.beginTransmission(slaveAddress);
  TinyWireM.write(highByte(addr));
  TinyWireM.write(lowByte(addr));
  TinyWireM.write(val);
  rc = TinyWireM.endTransmission();
  _delay_ms(5);
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
