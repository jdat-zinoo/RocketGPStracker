////////////////////////////////////////////////////////////////////////////////////////////////////
//  Configuration
////////////////////////////////////////////////////////////////////////////////////////////////////
//select GPS module
#define a2235h
//#define org1411

#define maxPower 31   //maximum tx power=-18dBm + value

const unsigned long onTx=5;     //transmiiter on time in minutes
const unsigned long offTx=2;    //transmitter off time in minutes

const int kBaudRate = 300;       //RTTY baud rate
const int beepFreq=50;        //beeper tone frequncy. used only when passive beeper

//Beeper time config
//when GPS OK
const uint8_t onTimeGood=10;    //1 sec on
const uint8_t offTimeGood=50;   //5 sec off
//when GPS bad
const uint8_t onTimeBad=5;      //0.5sec on
const uint8_t offTimeBad=5;     //0.5sec on

/*
 * Atmega fuses: (E:FE, H:DA, L:FF)
 * Externeal qautz oscillatopr 8 MHz, 3.3V, Brownout off
 * No Watchog!!!
 * Arduino pro mini bootloader
*/
////////////////////////////////////////////////////////////////////////////////////////////////////
//  System code
////////////////////////////////////////////////////////////////////////////////////////////////////

//chech for GPS module definition corectness

#ifndef org1411
  #ifndef a2235h
    #error "No GPS defined!"
  #endif
#endif

#ifdef org1411
  #ifdef a2235h
    #error "Define only one GPS!"
  #endif
#endif

// Not avialable now.
// Manual settings in registers
// for A2235H 433.92
// for ORG1411H 433.95
//frequnecy depends on GPS module to not intrefer with each other
//#ifdef org1411
//  #define frequency 435.02f
//#endif
//
//#ifdef a2235h
//  #define frequency 433.92f
//#endif

const  uint16_t maxAge=5000;    //maximum 5 seconds to wait for GPS serial data
const  uint8_t minSats=3;       //minimums Satelittes in GPS view for good fix

#include "fsk.h"
#include "RFM69OOK.h"
#include "RFM69OOKregisters.h"
#include <EEPROM.h>
#include "TinyGPS.h"

//const int txpin = A2;     //RTTY pin
#define txDir DDRC
#define txPort PORTC
#define txPin PINC2

//const int pinBeeper = 6;  // beeper
#define beepDir DDRD
#define beepPort PORTD
#define beepPin PIND6

#define radioCsPin 9
#define radioResetPin 2


TinyGPS gps;

FSKTransmitter tx;
RFM69OOK radio(radioCsPin);

//static uint8_t brDiv;   //for variable baudrate

char line[80];

//beeper variables
volatile uint16_t g100ms;
volatile uint16_t nextEvent;
volatile uint16_t onTime;
volatile uint16_t offTime;
volatile uint8_t beepOn;

//gps variables
float flat;
float flon;
float falt;
unsigned long age;
unsigned char numSats;

// tx time variables
unsigned long onTxTime;
unsigned long offTxTime;
unsigned long txEvent;
uint8_t txOn;

//tx power variables
uint8_t curPower;
uint8_t newMaxPower;

//GPS module dependent includes and config data
#ifdef a2235h
  // set to NMEA 9600
  const char sirfNMEA[]={0xA0, 0xA2, 0x00, 0x18, 0x81, 0x02, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x05, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x25, 0x80, 0x01, 0x3A, 0xB0, 0xB3};
#endif

void powerCal(){  
  curPower=EEPROM.read(0);  //read from eeprom to get power before restart
  EEPROM.update(0,0xff);    //write to eeprom reset marker. meaning: set to max power and wait delay for user to get chance to set max power
  beepOn=1;       //beep on
  delay(3000);    //wait 3 sec  
  beepOn=0;       //beep off
  
  if (curPower>maxPower) {
    newMaxPower=maxPower;   //limit number to max power. override user reset value
  }
  else {
    newMaxPower=curPower-1; // woops. we got restart. decrease max power
  }

  // for loop to ram power
  for (curPower=0; curPower<=newMaxPower; curPower++){
    uint8_t wr=curPower;
    if (curPower>0) {   //if power > 0 then temorary var wr=power-1 for storage on next reboot
      wr=curPower-1;
    } else {
      wr=0;             //if power already 0 then stay at 0
    }
    
    EEPROM.update(0,wr);  //protected store lower power for next reboot
    //  set TX power
    radio.writeReg(REG_OCP,0x10111);
    radio.writeReg(REG_TESTPA1,0x55);
    radio.writeReg(REG_TESTPA2,0x70);
    radio.writeReg(REG_PALEVEL,( 0b01100000 | curPower ) );
    radio.transmitBegin();    
    delay(100); //wait ramp timeout
    EEPROM.update(0,curPower);  // if we are here than everything OK. store working power.
  }
  
}

void setup()
{ 
  //Serial.begin(9600);

  // f**k. init here Tx On and Off time
  onTxTime=onTx*60*1000;
  offTxTime=offTx*60*1000;
  
  // Setup Timer2: CTC, prescaler 1024, irq on compare1 called baudRate times per second
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
  OCR2A = (F_CPU / 1024 / kBaudRate) - 1;
  //OCR2A=OCR2A_50;     //for variable baudrate
  TIMSK2 = _BV(OCIE2A);
  
  // Setup Timer1: CTC, prescaler 64, irq on compare1, called beeFreq times per second
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10);  //_BV(CS12) | _BV(CS10);
  OCR1A = (F_CPU  / 64 / (beepFreq*2)) - 1;
  TIMSK1 = _BV(OCIE1A);

  //beeper pin to output
  sbi(beepDir,beepPin);

  //Radio init stuff
  //Reset radio
  sbi(txDir,txPin);  
  pinMode(radioResetPin,INPUT_PULLUP);
  delay(10);
  pinMode(radioResetPin,INPUT);
  delay(10);  



  
  //init radio with libary functions
  radio.initialize();  

 
  //Set radio frequency manual
  #ifdef org1411
    radio.writeReg(REG_FRFMSB,0x6C);
    radio.writeReg(REG_FRFMID,0x7C);
    radio.writeReg(REG_FRFLSB,0xCC);
  #endif  
  
  #ifdef a2235h 
    radio.writeReg(REG_FRFMSB,0x6C);
    radio.writeReg(REG_FRFMID,0x7A);
    radio.writeReg(REG_FRFLSB,0xE1);
  #endif  
  
  // +/- deviation freq = 61*register (6*61=366Hz, RTTY FSK shift=732 hz
  radio.writeReg(REG_FDEVMSB,0);
  radio.writeReg(REG_FDEVLSB,6);      

  //radio.writeReg(REG_PARAMP,0);
  radio.writeReg(REG_PARAMP,0b1111);
    
  //rtty pin to output
  sbi(txPort,txPin);
  
  //set TX power
  powerCal();
  
  // GPS module dependent initialisation
  #ifdef org1411
    Serial.flush();
    Serial.begin(4800);
  #endif
  
  #ifdef a2235h 
    Serial.flush();
    Serial.begin(115200);
    Serial.write(sirfNMEA,sizeof(sirfNMEA));
    Serial.flush();
    Serial.begin(9600);
  #endif
  
  txEvent=millis()+onTxTime;    //init tx on/off event variable
  txOn=1;                       //start with TX on
}

void loop()
{

//  for(;;){
//    sbi(txPort,txPin);
//    delay(1000);
//    cbi(txPort,txPin);
//    delay(1000);
//
//  }
  
  // Read all characters from serial buffer and decode GPS data
  //this part is module independent because SiRFGPS and TinyGPS patched for identical interface
  while (Serial.available()) {        
    if (gps.encode(Serial.read())) {
      // Update position and altitude
      gps.f_get_position(&flat, &flon, &age);
      falt = gps.f_altitude();
      numSats = gps.satellites();
    }
  }  

  // tx on/off logic
//  if ( millis() > txEvent ) {
//    if (txOn==1) {
//      txOn=0;
//      txEvent=millis()+offTxTime;
//    } else {
//      txOn=1;
//      txEvent=millis()+onTxTime;
//    }   
//  }

  // if transmitter not transmitting then sreate new sata string
  if (!tx.isBusy()) {
    if ( txOn==1 ){  
      // Create RTTY TX packet and transmit
      char buffLatitude[10];
      char buffLongitude[10];
      char buffAltitude[8];
  
      //snprintf(line, sizeof(line), "%sN %sE %s %hhu %hhu\n",
      snprintf(line, sizeof(line), "%s %s %s %hhu %hhu\n",
        dtostrf(flat, 0, 5, buffLatitude),
        dtostrf(flon, 0, 5, buffLongitude),
        dtostrf(falt, 0, 0, buffAltitude),
        numSats,
        age
      );
      radio.transmitBegin();
      tx.transmit(line, strlen(line));
    } else {
      radio.transmitEnd();
    }
  }

  //check GPS status for beeper and set correct beep times
  if ((numSats>=minSats) && (age<maxAge)){
    onTime=onTimeGood;  
    offTime=offTimeGood;
  } else {
    onTime=onTimeBad;  
    offTime=offTimeBad;
  }

  //beeper time logic
  if (nextEvent<=g100ms){      
      if (beepOn==0){
        beepOn=1;
        nextEvent=g100ms+onTime;
      } else {
        beepOn=0;
        nextEvent=g100ms+offTime;
      }    
  }

//  //for variable baudrate
//  static uint16_t oldMs;
//  if (oldMs!=g100ms){  
//    if ( (g100ms % 600) == 0 ){
//      if (brDiv==brDiv_50){
//        brDiv=brDiv_300;
//        OCR2A=OCR2A_300;
//      } else {
//        brDiv=brDiv_50;
//        OCR2A=OCR2A_50;
//      }
//      //delay(100);
//      oldMs=g100ms;
//    }
//    
//  }
}

//timer1 interrupt for beeper stuff
ISR (TIMER1_COMPA_vect){
  static uint8_t flip;    // local varaible to trace pin state (can be optimized)
//  for passive beeper
//  if (beepOn==1){  
//    if (flip==0){
//      flip=1;      
//      cbi(beepPort,beepPin); //digitalWrite(pinBeeper,LOW);
//    } else {
//      flip=0;      
//      sbi(beepPort,beepPin); //digitalWrite(pinBeeper,HIGH);    
//    }
//  }

//  for active beeper
  if (beepOn==1){  
      sbi(beepPort,beepPin); //digitalWrite(pinBeeper,HIGH);    
    } else {
      cbi(beepPort,beepPin); //digitalWrite(pinBeeper,LOW);
    }
}

//timer2 interrupt for rtty stuff and 100 ms counter 
ISR(TIMER2_COMPA_vect) {
  static uint16_t cnt;
  tx.tick();
  cnt++;
  if (cnt >= kBaudRate/10) {
  //if (++cnt >= brDiv) {       //for wariable baudrate
    cnt = 0;
    g100ms++;
  }
}
