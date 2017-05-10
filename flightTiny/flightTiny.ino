/*************************************************************************
 Hardware description

 AtTiny85, magnetometer (MAG3110), eject output pin, buzzer, LED.
 The buzzer output pin also senses battery voltage via a resistor divider.
 The eject output serves also as a safe/flight pin.
 ATTiny85 is connected via USI bus to an I2C slave device (MAG3110).
 
 AtTiny85 fuses:
 LFUSE 	= 0xE2
 HFUSE 	= 0xDD
 XFUSE 	= 0xFF
*************************************************************************/

//#define DEBUG   1

#include <EEPROM.h>
#include "TinyWireM.h"

#include <avr/sleep.h>

#include "tiny.h"
#include "I2C.h"
#include "MAG3110.h"
#include "MS5607.h"
#include "flash.h"

const int kDitLength = 150 / 25;  // 10 wps

const int kLOG_SIZE = 160;      // 3 bytes per entry

const uint16_t flashSize=1000;  //entries in I2C EEPROM

const int kSERVO_PERIOD_MS  = 20;
const int kSERVO_ON_mS      = 1000;

const uint8_t kTIMER1_DIV512  = 0b1010;
const uint8_t kTIMER1_DIV1024 = 0b1011;
const uint8_t kTIMER1_PERIOD = F_CPU / 1024 * kSERVO_PERIOD_MS / 1000;

DigitalOut<PortB::pin4> led;
DigitalOut<PortB::pin3> pyro;
DigitalOut<PortB::pin1> buzzer;

TextOutStream<SoftwareSerialOut<DigitalOut<PortB::pin4>, 9600> > dbg;

const char *endl = "\r\n";

flash ee;
I2C bus;
BarometerSimple baroSensor(bus);
MAG3110 magSensor;

/*
//void testI2C();
//void testLED();
//void flashID(uint8_t id);

//void initPyro();
//void initSensors();
//void resetCalibration();

//void calibrate(int16_t mx, int16_t my, int16_t mz);
//void calibrateBaro(uint16_t p);

//bool checkFlightPin();
//void errorHalt();
//void clearLog();
//bool reportLog(uint16_t index);
//void updateLog(uint16_t index);

//void setMaxAltitude(uint16_t altitude);

//uint8_t lookupMorse(char c);

//void restoreState();
//void saveState();
*/
struct LogEntry {
  uint8_t   mag;
  uint16_t  altitude;
};

struct i2cLogEntry {
  uint8_t   stateLog;
  uint16_t  pressure;
  uint16_t  magX;
  uint16_t  magY;
  uint16_t  magZ;
  uint8_t   magT;
};


enum {
  kSTATE_SAFE,
  kSTATE_FLIGHT
};

enum {
  kBUZZER_TEST,
  kBUZZER_SILENT,
  kBUZZER_DESCENT,
  kBUZZER_ASCENT
};

enum {
  kFLAG_MEASURE,
  kFLAG_CAL,
  kFLAG_SERVO
};

int16_t gMinY, gMaxY, gZeroY;
uint16_t gZeroPressure;

uint8_t gState;
uint8_t gIsDescent;
bool    gPyroOn;
uint8_t gBuzzerMode;
uint16_t gAltitude;
uint16_t gMaxAltitude;
uint8_t  gLogIndex;

uint8_t  gMessage[6];

volatile uint8_t gFlags;

uint16_t gFlashIndex;

void setup()
{
  TinyWireM.begin();
  dbg.begin();

  dbg << "Reset" << endl;

  // Setup measurement timer
  //TCCR1 = _BV(CTC1) | kTIMER1_DIV512;
  TCCR1 = kTIMER1_DIV512;
  OCR1C = 124;
  OCR1A = 124;
  TIMSK |= _BV(OCIE1A);

  sei();

  // Beep buzzer to indicate reset
  gBuzzerMode = kBUZZER_TEST;
  _delay_ms(100);
  gBuzzerMode = kBUZZER_SILENT;


  //initPyro();
  initSensors();

  testI2C();
  
  restoreState();

  if (checkFlightPin()) {
    gState = kSTATE_FLIGHT;
  }
  else {
    gState = kSTATE_SAFE;
    resetCalibration();
    setMaxAltitude(0);
  }

  _delay_ms(100);
  gBuzzerMode = kBUZZER_TEST;
  _delay_ms(100);
  gBuzzerMode = kBUZZER_SILENT;
  _delay_ms(500);
}

class StringBuf {
public:
  StringBuf() : _buffer(0), _length(0), _capacity(0) {}
  StringBuf(char *buffer, uint8_t capacity) { assign(buffer, capacity); }

  void assign(char *buffer, uint8_t capacity) {
    _buffer = buffer;
    _length = 0;
    _capacity = capacity;
  }

  void write(uint8_t b) {
    if (_length < _capacity) {
      _buffer[_length] = b;
      _length++;
    }
  }

private:
  char *_buffer;
  uint8_t _length;
  uint8_t _capacity;
};

void loop()
{
  static uint8_t  pyroTimer;
  static bool     pyroPhase;

  if (bit_check(gFlags, kFLAG_MEASURE)) {
    bit_clear(gFlags, kFLAG_MEASURE);

    // Get sensor measurements
    int16_t   mx, my, mz;
    uint16_t  pressure, pressureComp;
    bool magOK = magSensor.readMag(mx, my, mz);
    bool baroOK = baroSensor.update();

    // Calculate altitude wrt to zero ground pressure
    if (baroOK) {
      pressure = baroSensor.getPressure();

      //dbg << pressure <<endl;

      if (gZeroPressure != 0) {
        pressureComp = (uint32_t)pressure * 25325 / gZeroPressure;
      }
      else {
        pressureComp = pressure;
      }
      gAltitude = baroSensor.getAltitude(pressureComp);
    }
/*
    dbg << pressureComp << " / " << gAltitude
        << " (max " << gMaxAltitude << "/" << (const char *)gMessage << ")"
        << " * " << mx << " / " << my << " / " << mz << " | " << gZeroY << endl;
*/
/*
    dbg << "Press: " <<pressureComp << " Alt: " << gAltitude
      << " * MagY: " << my << " | MagZero: " << gZeroY << " / State: " << gState << endl;
*/
dbg << mx << "," << my << "," << mz << "," << gZeroY << endl;

    bool doEject = magOK && (my > gZeroY);

    if (gState == kSTATE_SAFE) {
      // SAFE state
      // * indicate eject event on the buzzer
      // * calibrate magnetic sensor range
      // * calibrate reference (ground) pressure value
      // * dump log data on serial port (LED)

      gBuzzerMode = doEject ? kBUZZER_TEST : kBUZZER_SILENT;

      if (magOK) calibrate(mx, my, mz);
      if (baroOK) calibrateBaro(pressure);

      if (gLogIndex >= kLOG_SIZE) {
        gLogIndex = 0;  // Start all over again
      }
      if (reportLog(gLogIndex)) {
        gLogIndex++;
      }
      else {
        gLogIndex = 0;
      }

      // Check if state has changed
      if (checkFlightPin()) {
        gState = kSTATE_FLIGHT;
        setMaxAltitude(0);
        gIsDescent = false;
        saveState();
        clearLog();
        gLogIndex = 0;
      }
    }
    else if (gState == kSTATE_FLIGHT) {
      // FLIGHT state
      // * manage eject (repeat 3 times)
      // * compute altitude
      // * update and store log data

      if (gAltitude > gMaxAltitude) {
        setMaxAltitude(gAltitude);
      }

      if (doEject) {
        //if (!gPyroOn)
        {
          gIsDescent = true;

          pyroPhase = !pyroPhase;   // alternate between on/off states
          gPyroOn = pyroPhase;
          pyroTimer = 125;
        }
      }
      else {
        pyroPhase = false;
        if (gPyroOn) {
          gPyroOn = false;
          pyroTimer = 125;
        }
      }

      gBuzzerMode = gIsDescent ? kBUZZER_DESCENT : kBUZZER_ASCENT;

      if (gLogIndex < kLOG_SIZE) {
        updateLog(gLogIndex++);
      }

      // Check if state has changed
      if (!checkFlightPin()) {
        gState = kSTATE_SAFE;
        resetCalibration();
        gPyroOn = false;
        pyroTimer = 125;
      }
    }
  }


  if (bit_check(gFlags, kFLAG_SERVO)) {
    bit_clear(gFlags, kFLAG_SERVO);

    if (pyroTimer) {
      pyroTimer--;

      pyro.high();
      if (gPyroOn) _delay_ms(kSERVO_ON_mS);
      //else _delay_us(kSERVO_OFF_US);
      pyro.low();
    }
    else {
      pyro.low();
    }
  }

  //delay(1000);
  sleep_mode();
}

ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0;
  static uint8_t cnt1;
  static uint8_t cnt2;

  static uint8_t msgIdx;
  static uint8_t ditIdx;
  static uint8_t ditTimer;
  static uint8_t morseCode;
  static uint8_t ditCount;
  static uint8_t ditState;

  if (++cnt1 >= 30) {
    cnt1 = 0;
    bit_set(gFlags, kFLAG_SERVO);
  }

  uint8_t measurePeriod = (gState == kSTATE_SAFE) ? 25 : 125;
  //uint8_t measurePeriod = (gState == kSTATE_SAFE) ? 25 : 75;
  if (++cnt2 >= measurePeriod) {
    cnt2 = 0;
    bit_set(gFlags, kFLAG_MEASURE);
  }

  switch (gBuzzerMode) {
  case kBUZZER_SILENT:
    buzzer.low();
    break;
  case kBUZZER_TEST:
    buzzer.high();
    break;
  case kBUZZER_ASCENT:
    if (cnt2 < 10 || (cnt2 > 20 && cnt2 < 40)) buzzer.high();
    else buzzer.low();

    break;
  case kBUZZER_DESCENT:
    if (ditTimer) ditTimer--;
    else {
      if (ditState == 1) {
        // Intercharacter silence
        ditState = 0;
        ditTimer = (ditIdx == 0) ? 3*kDitLength : kDitLength;
        if (ditIdx == 0 && gMessage[msgIdx] == 0) {
          msgIdx = 0;
          ditTimer = 12*kDitLength;
        }
      }
      else {
        if (ditIdx > 0) {
          ditIdx--;
          morseCode >>= 1;
          ditTimer = (morseCode & 1) ? 3*kDitLength : kDitLength;
          ditState = 1;
        }
        else {
          // look up morse code
          morseCode = lookupMorse(gMessage[msgIdx]);
          ditCount = morseCode >> 5;
          //morseCode = 0b00000010;
          //ditCount = 5;

          if (ditCount > 0) {
            ditIdx = ditCount - 1;
            ditState = 1;
            ditTimer = (morseCode & 1) ? 3*kDitLength : kDitLength;
          }

          msgIdx++;
        }
      }
    }

    // Dit/deet
    if (ditState == 1) {
      buzzer.high();
    }
    else {
      buzzer.low();
    }
    break;
  }
}

void calibrate(int16_t mx, int16_t my, int16_t mz)
{

  //static int16_t minx = 32000, miny = 32000, minz = 32000;
  //static int16_t maxx = -32000, maxy = -32000, maxz = -32000;

  //if (mx > maxx) maxx = mx;
  //else if (mx < minx) minx = mx;
  //if (my > maxy) maxy = my;
  //else if (my < miny) miny = my;
  //if (mz > maxz) maxz = mz;
  //else if (mz < minz) minz = mz;

  //int16_t cx = (minx + maxx) / 2;
  //int16_t cy = (miny + maxy) / 2;
  //int16_t cz = (minz + maxz) / 2;


  //dbg << "Cal: [" << cx << "/" << cy << "/" << cz << "]" << endl;
  //dbg << mx << "," << my << "," << mz << "," << cx << "," << cy << "," << cz << endl;
  //dbg <<  my << ","  << cy  << endl;

  if (my < gMinY) gMinY = my;
  if (my > gMaxY) gMaxY = my;
  gZeroY = (gMinY + gMaxY) / 2;

  //int16_t a = (gMinY + gMaxY) / 2;
  //gZeroY=a+(gMaxY-a)/2;

  //gZeroY=gZeroY+gZeroY/2;
  //gZeroY = (gMinY + gMaxY) / 4;

  //dbg <<  my << ","  << gZeroY  << endl;
}

void calibrateBaro(uint16_t pressure)
{
  static uint32_t  sum;
  static uint16_t  count;

  sum += pressure;
  count++;

  gZeroPressure = sum / count;
}

void resetCalibration()
{
  gMinY = 32000;
  gMaxY = -32000;
  gZeroPressure = 0;
}

bool checkFlightPin() {
  bool result;
  ADCSRA = _BV(ADPS2) | _BV(ADEN);

  pyro.low();
  pyro.modeInput();
  _delay_ms(1);

  ADMUX = 3;
  bit_set(ADCSRA, ADSC);
  while (bit_check(ADCSRA, ADSC)) {}
  uint16_t adcValue = ADC;

  //dbg << "ARM: " << adcValue <<endl;

  result = (adcValue > 50);

  //pyro.readAnalog();
  pyro.modeOutput();
  ADCSRA = 0;

  return result;

  /*
  led.high();
  _delay_us(100);
  return (led.read() != 0);
  */
}

void errorHalt() {
  while (true) {
    buzzer.high();
    _delay_ms(100);
    buzzer.low();
    _delay_ms(100);
  }
}

void initSensors() {
  bool magReady = false;
  for (uint8_t nTry = 10; nTry > 0; nTry--) {
    dbg << "Initializing magfield sensor" << endl;
    magReady = magSensor.begin();
    if (magReady) {
      magSensor.configure1(0b00000001);  //80 Hz sampilg, active mode
      magSensor.configure2(0b10100000);  //auto reset (need test), raw mode
      magSensor.mode(0b00000001);  //active mode RAW
      dbg << "Magnetometer ready" << endl;
      break;
    }
    delay(500);
  }
  if (!magReady) {
    errorHalt();
  }

  bool baroReady = false;
  for (uint8_t nTry = 10; nTry > 0; nTry--) {
    dbg << "Initializing pressure sensor" << endl;
    baroReady = baroSensor.initialize();
    if (baroReady) {
      dbg << "Barometer ready" << endl;
      break;
    }
    delay(500);
  }
  if (!baroReady) {
    errorHalt();
  }
}
/*
void initPyro() {
  for (uint8_t cnt = 0; cnt < 25; cnt++) {
    //pyro.high();
    //delay(kSERVO_OFF_US);
    //pyro.low();
    //delay(kSERVO_PERIOD_MS);
  }
}
*/
void testLED()
{
  led.high();
  _delay_ms(500);
  //pyro.high();
  //_delay_ms(100);
  //pyro.low();

  led.low();
  _delay_ms(1000);
}

void flashID(uint8_t id)
{
  for (uint8_t mask = 0x80; mask > 0; mask >>= 1) {
    if (id & mask) {
      //pyro.high();
    }
    else {
      led.high();
    }
    _delay_ms(400);
    led.low();
    //pyro.low();
    _delay_ms(100);
  }
}


void testI2C() {
  //SoftwareI2C<PortB::pin0, PortB::pin2> bus;
  uint8_t a;
  byte rc;
  rc=ee.begin();
  //rc=ee.writeByte(5,0x55);
  //dbg << "FLASH wr: " << rc << endl;
  //rc=ee.readByte(5,a);  
  //dbg << "FLASH rd: " << rc << "\t" << a << endl;

  for (uint16_t i=1;i!=0;i++){
    //rc=ee.writeByte(i,lowByte(i));
    rc=ee.readByte(i,a);  
    dbg << "FLASH rd: " << i << "\t" << a << "\t" << rc <<endl;
  }
  errorHalt();
  
/*
test.modeOutput();
while (true){
  test.high();
  delay(2000);
  test.low();
  delay(2000);
  }
  errorHalt();
*/
/*
  uint8_t slaveAddress = 0b0011110;
  //uint8_t rc;
  for (uint8_t i=0;i<127;i++){
    slaveAddress=i;
    TinyWireM.beginTransmission(slaveAddress);
    TinyWireM.send(0x0);
    rc = TinyWireM.endTransmission();
    if (rc != 0) {
      dbg << slaveAddress << endl;
    }
    else {
      dbg << slaveAddress << " Have" << endl;
      //rc = TinyWireM.requestFrom(slaveAddress, 1); // Request 1 byte from slave
      //if (rc != 0) {
      //  dbg << "Failed to read from slave: " << i << endl;
      //}
    }
    //if (TinyWireM.available() > 0) {
    //  uint8_t id = TinyWireM.receive();
    //  dbg << "ID: " << id << endl;
    //}
  }
errorHalt();
*/
/*
//char s[5];
byte a;
  for (byte i=8;i<127;i++){
    //bus.read(i<<1, s, 0);
    TinyWireM.beginTransmission(i);
    a=TinyWireM.endTransmission();
    dbg << "I: " << i << " data: " <<  a <<endl;
  }
  errorHalt();
*/
}

void clearLog()
{
  /*
  for (uint16_t idx = 0; idx < 512; idx++) {
    EEPROM.write(idx, 0xFF);
  }
  */

  uint8_t idx = 0;
  LogEntry entry;
  entry.altitude = 0xFFFF;
  EEPROM.put(10 + 3 * idx, entry);
}

void updateLog(uint16_t idx)
{
  LogEntry entry;
  entry.mag = (gIsDescent ? 0x01 : 0x00);
  entry.altitude = gAltitude;
  EEPROM.put(10 + 3 * idx, entry);

  entry.altitude = 0xFFFF;
  EEPROM.put(10 + 3 * (idx + 1), entry);
}

bool reportLog(uint16_t idx)
{
  LogEntry entry;
  EEPROM.get(10 + 3 * idx, entry);
  if (entry.altitude != 0xFFFF) {
    dbg << idx << '\t' << entry.mag << '\t' << entry.altitude << endl;
    return true;
  }
  return false;
}

void restoreState()
{
  uint16_t maxAlt = 0;
  // Find the last entry in EEPROM
  uint16_t idx = 0;
  while (idx < kLOG_SIZE) {
    LogEntry entry;
    EEPROM.get(10 + 3 * idx, entry);
    if (entry.altitude == 0xFFFF) break;

    if (entry.altitude > maxAlt) {
      maxAlt = entry.altitude;
    }
    if (entry.mag & 1) gIsDescent = true;

    idx++;
  }
  gLogIndex = idx;
  setMaxAltitude(maxAlt);

  // restore gMaxAltitude
  //EEPROM.get(0, gMaxAltitude);
  //if (gMaxAltitude == 0xFFFF) gMaxAltitude = 0;
  EEPROM.get(0, gZeroPressure);
  EEPROM.get(2, gZeroY);
}

void saveState()
{
  // save gMaxAltitude
  //EEPROM.put(0, gMaxAltitude);

  EEPROM.put(0, gZeroPressure);
  EEPROM.put(2, gZeroY);
}

uint8_t lookupMorse(char c)
{
  if (c < '0' || c > '9') return 0;

  uint8_t idx = 0;
  uint8_t code = 0;
  uint8_t length = 5;
  uint8_t digit = c - '0';
  if (digit <= 5) {
    for (; idx < 5 - digit; idx++) {
      code <<= 1;
      code |= 1;
    }
    for (; idx < 5; idx++) {
      code <<= 1;
      code |= 0;
    }
  }
  else {
    for (; idx < 10 - digit; idx++) {
      code <<= 1;
      code |= 0;
    }
    for (; idx < 5; idx++) {
      code <<= 1;
      code |= 1;
    }
  }
  code |= (length << 5);
  return code;
}

void setMaxAltitude(uint16_t altitude) {
  gMaxAltitude = altitude;

  // Convert maximum altitude to decimal form
  TextOutStream<StringBuf> gMessageStr;
  gMessageStr.assign((char *)gMessage, 6);
  gMessageStr << gMaxAltitude << '\0';
}
