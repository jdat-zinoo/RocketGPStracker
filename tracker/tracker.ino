////////////////////////////////////////////////////////////////////////////////////////////////////
//  Configuration
////////////////////////////////////////////////////////////////////////////////////////////////////
//const int kBaudRate = 300;
const int kBaudRate = 50;       //RTTY baud rate
const int beepFreq=2500;        //beeper tome frequncy

//when GPS OK
const uint8_t onTimeGood=10;    //1 sec on
const uint8_t offTimeGood=50;   //5 sec off

//when GPS bad
const uint8_t onTimeBad=5;      //0.5sec on
const uint8_t offTimeBad=5;     //0.5sec on

const  uint16_t maxAge=5000;    //maximum 5 seconds to wait for GPS serial data
const  uint8_t minSats=3;       //minimums Satelittes in GPS view for good fix

//select GPS module
#define a2235h
//#define org1411



/*
 * Atmega fuses: (E:FE, H:DA, L:FF)
 * Externeal qautz oscillatopr 8 MHz, 3.3V, Brownout 1.8V
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

//frequnecy depends on GPS module to not intrefer with each other
#ifdef org1411
  #define frequency 433.95f
#endif

#ifdef a2235h
  #define frequency 433.92f
#endif


#include "fsk.h"
#include "RFM69OOK.h"
#include "RFM69OOKregisters.h"

//const int txpin = A1;     //RTTY pin
#define txDir DDRC
#define txPort PORTC
#define txPin PINC1

//const int pinBeeper = 6;  // beeper
#define beepDir DDRD
#define beepPort PORTD
#define beepPin PIND6

#define radioCsPin 9
#define radioResetPin 2

FSKTransmitter tx;

RFM69OOK radio(radioCsPin,3,true);    //high power +20 dBm
//RFM69OOK radio(radioCsPin);         //low power +0dBm

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

//int years;
//byte months;
//byte days;
//byte hours;
//byte minutes;
//byte seconds;
//byte hundredths;


//GPS module dependent includes and config data
//#include "TinyGPS.h"
//TinyGPS gps;

#ifdef org1411
  //const char sirfBaud[]={0xA0, 0xA2, 0x00, 0x09, 0x86, 0x00, 0x00, 0x25, 0x80, 0x08, 0x01, 0x00, 0x00, 0x01, 0x34, 0xB0, 0xB3};
  //const char sirfNMEA[]={0xA0, 0xA2, 0x00, 0x02, 0x87, 0x02, 0x00, 0x89, 0xB0, 0xB3};
#endif

#ifdef a2235h
  #include "SirfGPS.h"  
  SirfGPS gps;
  //#include "TinyGPS.h"
  //TinyGPS gps;
  const char sirfBaud[]={0xA0, 0xA2, 0x00, 0x09, 0x86, 0x00, 0x00, 0x25, 0x80, 0x08, 0x01, 0x00, 0x00, 0x01, 0x34, 0xB0, 0xB3};
  //const char sirfNMEA[]={0xA0, 0xA2, 0x00, 0x02, 0x87, 0x02, 0x00, 0x89, 0xB0, 0xB3};
#endif

void setup()
{ 

// GPS module dependent initialisation
#ifdef org1411
  //Serial.begin(115200);
  //Serial.write(sirfBaud,sizeof(sirfBaud));
  //Serial.flush();
  //Serial.begin(4800);
  //Serial.write(sirfBaud,sizeof(sirfBaud));  
  //Serial.write(sirfNMEA,sizeof(sirfNMEA));
#endif

#ifdef a2235h 
  Serial.flush();
  Serial.begin(115200);
  Serial.write(sirfBaud,sizeof(sirfBaud));
  Serial.flush();
  Serial.begin(9600);
  Serial.write(sirfBaud,sizeof(sirfBaud));  
  //Serial.write(sirfNMEA,sizeof(sirfNMEA));
#endif

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

  //beeper to output
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
  radio.setFrequencyMHz(frequency);  
  // +/- deviation freq = 61*register (6*61=366Hz, RTTY FSK shift=732 hz
  radio.writeReg(REG_FDEVMSB,0);
  radio.writeReg(REG_FDEVLSB,6);      

  //rtty pin to output
  sbi(txPort,txPin);
  radio.transmitBegin();
}

void loop()
{

  // Read all characters from serial buffer and decode GPS data
  //this part is module independent bause SiRFGPS and TinyGPS patched for identical interface
  while (Serial.available()) {        
    if (gps.encode(Serial.read())) {
      // Update position and altitude
      gps.f_get_position(&flat, &flon, &age);
      falt = gps.f_altitude();
      numSats = gps.satellites();
      //gps.get_datetime(&date, &time, &age);
      //gps.crack_datetime(&years, &months, &days, &hours, &minutes, &seconds, &hundredths, &age);
    }
  }  

  if (!tx.isBusy()) {
    // Create RTTY TX packet and transmit
    char buffLatitude[10];
    char buffLongitude[10];
    char buffAltitude[8];

    //snprintf(line, sizeof(line), "%s %s %s %hhu %hhu:%hhu:%hhu\n",
    snprintf(line, sizeof(line), "%sN %sE %s %hhu %hhu\n",
      dtostrf(flat, 0, 5, buffLatitude),
      dtostrf(flon, 0, 5, buffLongitude),
      dtostrf(falt, 0, 0, buffAltitude),
      numSats,
      //hours,
      //minutes,
      //seconds
      age
    );

    tx.transmit(line, strlen(line));
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
  if (beepOn==1){  
    if (flip==0){
      flip=1;      
      cbi(beepPort,beepPin); //digitalWrite(pinBeeper,LOW);
    } else {
      flip=0;      
      sbi(beepPort,beepPin); //digitalWrite(pinBeeper,HIGH);    
    }
  }
}

//timer2 interrupt for rtty stuff and 100 ms counter 
ISR(TIMER2_COMPA_vect) {
  static uint16_t cnt;
  tx.tick();
  cnt++;
  if (++cnt >= kBaudRate/10) {
  //if (++cnt >= brDiv) {       //for wariable baudrate
    cnt = 0;
    g100ms++;
  }
}
