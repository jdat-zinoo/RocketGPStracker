#pragma once

#include "Arduino.h"

// A1 pin
#define txDir DDRC
#define txPort PORTC
#define txPin PINC2

#define frequency 433.92f
#define radioCsPin 9
#define radioResetPin 2

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 

//#define bit_set(byte, bit)    ((byte) |= (1 << (bit)))
//#define bit_clear(byte, bit)  ((byte) &= ~(1 << (bit)))

class FSKTransmitter {
public:
  FSKTransmitter();

  void enable();
  void disable();
  void transmit(const uint8_t *buffer, uint16_t length);
  void tick();

  void mark();
  void space();

  bool isBusy();

private:
  const uint8_t * txBuffer;
  uint16_t  txLength;
  uint8_t   bitIndex;
  uint8_t   txShift;
  bool      autoShutdown;
  bool      active;

  void shiftNew();
};

