/*************************************************************************
 Hardware description

 AtTiny85, magnetometer (MAG3110), eject output pin, buzzer, LED.
 The eject output serves also as a safe/flight pin.
 ATTiny85 is connected via USI bus to an I2C slave devices (MAG3110).
 
 AtTiny85 fuses:
 LFUSE 	= 0xE2
 HFUSE 	= 0xD6
 XFUSE 	= 0xFF

 * Internal RC @ 8MHz
 * Brownout<1.8v
 * Preserve EEPROM memory
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

const int kLOG_SIZE = 160;      // 3 bytes per entry


const uint8_t kTIMER1_DIV4096 = 0b1101;

DigitalOut<PortB::pin3> pyro;
DigitalOut<PortB::pin1> led;

TextOutStream<SoftwareSerialOut<DigitalOut<PortB::pin4>, 9600> > dbg;

const char *endl = "\r\n";

I2C bus;
BarometerSimple baroSensor(bus);
MAG3110 magSensor;
flash ee;

//uint16_t flashSize;  //entries in I2C EEPROM
const uint16_t flashSize=65536/sizeof(ee.entry);  //entries in I2C EEPROM
//const uint16_t flashSize=50;  //entries in I2C EEPROM


// structure for restart protected data
struct _stateData {
  uint16_t zeroPressure;        // pressure on launchpad
  int16_t zeroY;                // magnetometer midpoint while calibrating
  uint16_t flashIndex;          // last entry in I2C EEPROM flash
  uint16_t maxAltitude;         // max Altitude
  volatile uint8_t isDescent;   // for some unknown state ??? must be resolved
};

// arm pin state
enum {
  kSTATE_SAFE,
  kSTATE_FLIGHT
};

// buzzer state
enum {
  kBUZZER_TEST,
  kBUZZER_SILENT,
  kBUZZER_DESCENT,
  kBUZZER_ASCENT
};

// in SAFE mode calibrating. in FLIGHT mode measuring ??? Why? must be resolved
/*
enum {
  kFLAG_MEASURE,
  kFLAG_CAL
};
*/

// minimum and maximum for magnetometer Y axis
int16_t gMinY, gMaxY;

volatile uint8_t shoot=0;   // pyro shooting state. ??? dirty! must be resolved

volatile uint8_t gState;    // system state. ??? must be resolved
volatile uint8_t    gPyroOn=0;    // if this !=0 then pyro will shoot. Be careful
volatile uint8_t gBuzzerMode;     // buzzer state variable
uint16_t gAltitude;               // current altitude

volatile uint8_t gFlagMeasure;          // some system flags ??? must be resolved

_stateData stateData;             // restart protected data

// this will erase all eeprom contents. must be optimised
void clearFlash(){
    ee.entry.magX=0;
    ee.entry.magY=0;
    ee.entry.magZ=0;
    ee.entry.magT=0;
    ee.entry.pressure=0;
    ee.entry.altitude=0;
    ee.entry.stateLog=0;
    for (uint16_t i=0;i<flashSize;i++){
      ee.writeRecord(i);
      dbg << "Erase: " << i << endl;
    }
  
}

void setup()
{
  gPyroOn=0;          // just in case no shoot at this point
  
  TinyWireM.begin();  // init I2C
  dbg.begin();        // init serial output

  dbg << "Reset" << endl;   // we are booting
  
  //dbg << sizeof(ee.entry) << endl;
  //dbg << flashSize << endl;
  
  //clearFlash();
  
  // Setup measurement timer. 10 IRQs per second
  TCCR1 = _BV(CTC1) | kTIMER1_DIV4096;
  //TCCR1 = kTIMER1_DIV512;
  OCR1C = 193;
  OCR1A = OCR1C;
  TIMSK |= _BV(OCIE1A);

  sei();

  initSensors();
  restoreState();
  //EEPROM.get(0, stateData);  //restoreState

  reportState();      // dump state data (internal eeprom) to serial
  
  dbg << "idx,state,press,alt,X,Y,Z,T" << endl;   // header for I2C EEPROM data. dirty hack
  if (checkFlightPin()==1) {
    gState = kSTATE_FLIGHT;
  }
  else {
    gState = kSTATE_SAFE;
    resetCalibration();       // something messy. neet to check
    //setMaxAltitude(0);
    //stateData.maxAltitude = 0;
  }
}

void loop()
{
  static uint16_t rFlashIndex;      //I2C EEPROM flash index for reading and data dumping only.

  //if (bit_check(gFlags, kFLAG_MEASURE)) {       //we got main instructions form timer to exeute main loop
  //  bit_clear(gFlags, kFLAG_MEASURE);           // reset flag, amd wait for next chance from timer
  if (gFlagMeasure==1) {
    gFlagMeasure=0;
    
    // Get sensor measurements
    int16_t   mx, my, mz;
    uint16_t  pressure, pressureComp;
    bool magOK = magSensor.readMag(mx, my, mz);
    bool baroOK = baroSensor.update();

    // store data in I2C EEPROm structure for logging
    ee.entry.magX=mx;
    ee.entry.magY=my;
    ee.entry.magZ=mz;
    ee.entry.magT=magSensor.readMagT();

    // Calculate altitude wrt to zero ground pressure
    if (baroOK) {
      pressure = baroSensor.getPressure();

      //dbg << pressure <<endl;

      if (stateData.zeroPressure != 0) {
        pressureComp = (uint32_t)pressure * 25325 / stateData.zeroPressure;
      }
      else {
        pressureComp = pressure;
      }
      gAltitude = baroSensor.getAltitude(pressureComp);
    }

    // store data in I2C EEPROm structure for logging
    ee.entry.pressure=pressureComp;
    ee.entry.altitude=gAltitude;

#ifdef DEBUG
dbg << mx << "," << my << "," << mz << "," << stateData.zeroY << endl;
//dbg << mx << "," << ee.entry.magX << "---" << my << "," << ee.entry.magY << "---" << mz << "," << ee.entry.magZ << endl;
#endif

    // this is tricky, must be fixed
    bool doEject = magOK && (my > stateData.zeroY);     //depending on electronic orientation use ">" or "<"

    // hack to store some system state in I2C EEPROM
    uint8_t xxx=0;
    xxx=(gState & 0x1)<<7;
    xxx=xxx | ((doEject & 0x1)<<6);
    xxx=xxx | ((gBuzzerMode & 0x3) <<4);
    xxx=xxx | ((stateData.isDescent & 0x1 ) <<3);
    ee.entry.stateLog=xxx;

    if (gState == kSTATE_SAFE) {
      // SAFE state
      // * indicate eject event on the led
      // * calibrate magnetic sensor range
      // * calibrate reference (ground) pressure value
      // * dump log data on serial port (LED)

      gBuzzerMode = doEject ? kBUZZER_TEST : kBUZZER_SILENT;

      if (magOK) calibrate(mx, my, mz);
      if (baroOK) calibrateBaro(pressure);

           
      // Check if state has changed
      if (checkFlightPin()==1) {
        gState = kSTATE_FLIGHT;
        stateData.maxAltitude=0;
        stateData.isDescent = false;
        //saveState();
        clearFlashLog(); //Also saves all state
        //gLogIndex = 0;
        
      }
    }
    else if (gState == kSTATE_FLIGHT) {
      // FLIGHT state
      // * manage eject (repeat 3 times)
      // * compute altitude
      // * update and store log data

      dbg << stateData.flashIndex <<  endl;
      if (gAltitude > stateData.maxAltitude) {
        stateData.maxAltitude=gAltitude;
      }

      if (doEject) {
        gPyroOn=1;
        stateData.isDescent = true;        
      }
      else {
        gPyroOn=0;
      }

      gBuzzerMode = stateData.isDescent ? kBUZZER_DESCENT : kBUZZER_ASCENT;

      //if (gLogIndex < kLOG_SIZE) {updateLog(gLogIndex++);}

      if (stateData.flashIndex<flashSize){
        updateFlashLog(stateData.flashIndex++);
        
      }
      saveState(); //automatically saves flash memory index
      //EEPROM.put(0, stateData);  //save State //automatically saves flash memory index
      
      // Check if state has changed
      if (checkFlightPin()==0) {
        gState = kSTATE_SAFE;
        resetCalibration();
        gPyroOn = 0;
        rFlashIndex=0;
        reportState();
        dbg << "idx,state,press,alt,X,Y,Z,T" << endl;
      }
    }
  }

  if (gState == kSTATE_SAFE) {
    if (rFlashIndex<stateData.flashIndex){
      reportFlashLog(rFlashIndex++);
    } else {
      rFlashIndex=0;
      reportState();
      dbg << "idx,state,press,alt,X,Y,Z,T" << endl;   // header for I2C EEPROM data. dirty hack
    }
    
  } else if (gState == kSTATE_FLIGHT) {
    sleep_mode();
  }
}

volatile uint8_t cnt2;

ISR(TIMER1_COMPA_vect) {
  //TCNT1 = 0;
  static uint8_t cnt1;
  
  if (++cnt1 >= 30) {
    cnt1 = 0;
  }
  
  if (++cnt2 >= 10) {
    cnt2 = 0;
  }
    //bit_set(gFlags, kFLAG_MEASURE);
    gFlagMeasure=1;

  if ( (gPyroOn!=0) && (cnt2 < 5) ) {
    pyro.high();
    shoot=1;
  } else {
    pyro.low();
    shoot=0;
  }
  
  switch (gBuzzerMode) {
  case kBUZZER_SILENT:
    led.low();
    break;
  case kBUZZER_TEST:
    led.high();
    break;
  case kBUZZER_ASCENT:
    if (cnt1 < 2 || (cnt1 > 8 && cnt1 < 15)) led.high();
    else led.low();
    break;
  case kBUZZER_DESCENT:    
    if (cnt1 < 2 || (cnt1 > 8 && cnt1 < 15)) led.high();
    else led.low();
    break;

  }
}

int checkFlightPin() {
  int result=-1;
  if (shoot==0){
      ADCSRA = _BV(ADPS2) | _BV(ADEN);
    
      
      pyro.low();
      pyro.modeInput();
      _delay_ms(1);
    
      ADMUX = 3;
      bit_set(ADCSRA, ADSC);
      while (bit_check(ADCSRA, ADSC)) {}
      uint16_t adcValue = ADC;
    
      //dbg << "ARM: " << adcValue <<endl;
    
      if (adcValue > 50){
        result = 1;
      } else {
        result = 0;
      }
    
      pyro.modeOutput();
      ADCSRA = 0; 
  }
  
  return result;

}

void errorHalt() {
  while (true) {
    led.high();
    _delay_ms(100);
    led.low();
    _delay_ms(100);
    dbg << "Halted!" <<endl;
  }
}

void calibrate(int16_t mx, int16_t my, int16_t mz)
{
  if (my < gMinY) gMinY = my;
  if (my > gMaxY) gMaxY = my;
  stateData.zeroY = (gMinY + gMaxY) / 2;

}

void calibrateBaro(uint16_t pressure)
{
  static uint32_t  sum;
  static uint16_t  count;

  sum += pressure;
  count++;

  stateData.zeroPressure = sum / count;
}

void resetCalibration()
{
  gMinY = 32000;
  gMaxY = -32000;
  stateData.zeroPressure = 0;
  stateData.maxAltitude = 0;
}


void initSensors() {
  bool magReady = false;
  for (uint8_t nTry = 3; nTry > 0; nTry--) {
    dbg << "Initializing magfield sensor" << endl;
    magReady = magSensor.begin();
    if (magReady) {
      magSensor.configure1(0b00000001);  //80 Hz sampling, active mode
      magSensor.configure2(0b10100000);  //auto reset (need test), raw mode
      magSensor.mode(0b00000001);  //active mode RAW
      dbg << "Magnetometer ready" << endl;
      break;
    }
    delay(200);
  }
  if (!magReady) {
    errorHalt();
  }

  bool baroReady = false;
  for (uint8_t nTry = 3; nTry > 0; nTry--) {
    dbg << "Initializing pressure sensor" << endl;
    baroReady = baroSensor.initialize();
    if (baroReady) {
      dbg << "Barometer ready" << endl;
      break;
    }
    delay(200);
  }
  if (!baroReady) {
    errorHalt();
  }
}

void clearFlashLog(){
  stateData.flashIndex=0;  
  saveState();
  //EEPROM.put(0, stateData);  //save State
}


void updateFlashLog(uint16_t idx){
  ee.writeRecord(idx);
  saveState();
}

bool reportFlashLog(uint16_t idx){
  ee.readRecord(idx);
  #ifndef DEBUG
  dbg << idx << ',' << ee.entry.stateLog << ',' << ee.entry.pressure << ',' <<ee.entry.altitude << ',' << ee.entry.magX << ',' << ee.entry.magY << ',' << ee.entry.magZ << ',' << ee.entry.magT <<endl;
  #endif
}

void reportState(){
  dbg << "zeroPressure,zeroY,flashIndex,maxAltitude" << endl;
  dbg << stateData.zeroPressure << "," << stateData.zeroY << "," << stateData.flashIndex << "," << stateData.maxAltitude << endl;  
}

void restoreState() {
  EEPROM.get(0, stateData);
}

void saveState() {
  EEPROM.put(0, stateData);
}

