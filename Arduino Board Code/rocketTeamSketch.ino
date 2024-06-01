//Includes
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_EEPROM_I2C.h"

//Defines
#define MinimumHeight 400; 

//Structures
struct startData {
  float groundAltitude;
  float groundPressure;
  float groundTemperature;
} ;

struct flightData {
  uint32_t time;
  uint32_t heightRate;
  uint32_t height;
} ;

struct stateMachine{
  uint8_t states;
  bool minimumHeight;
} ;

/*
*Function Initializations
*/

//Barometer
void initializeBarometerSleep();
void initializeBarometerNormal();
void printBarometerValues();
float barometerAltitude ();
float barometerHeight ();
void initGroundData();

//Accelerometer
void initializeAccelerometer();
void printAccelerometerValues ();

//Second Event
void setSecondEventCurrentImpulse();
void resetSecondEventCurrentImpulse();
void secondEvent();

//EEPROM
void initalizeEEPROM();
void eepromWriteState();
void eepromReadState();
void eepromWritePrimer();
void eepromReadPrimer();
void eepromWriteGroundData();
void eepromReadGroundData();

//Device Initializations
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
Adafruit_EEPROM_I2C i2ceeprom;


//Struct Initalizations
startData groundData;
stateMachine states;
#define FLIGHT_DATA_BUFFER_SIZE 5
flightData flightDataArray[FLIGHT_DATA_BUFFER_SIZE]; 

// States and Counters
// enum {SLEEP, AWAKE, FLY, READY, TRIGGERED,};
#define SLEEP 0
#define AWAKE 1
#define FLY 2
#define READY 3
#define TRIGGERED 4
int sleepCounter = 0;
int awakeCounter = 0;
int flyCounter = 0;
int readyCounter = 0;
int triggeredCounter = 0;

//Mach Delay
int machDelayA;
int machDelayB;
int machDelayTotal;

#define machDelay 1000

byte receivedMessage;
uint8_t counter;

void setup() {
  Serial.begin(9600);
  while ( !Serial ) delay(100);
  Wire.begin(0x08);
  Wire.onReceive(AVTransmission);
  initializeBarometerSleep();
  initializeAccelerometer();
  initializeEEPROM();
  delay(100);
  eepromReadState();
  eepromReadPrimer();
  if (states.states != 0) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);
  }
  if (states.states == 0) {
      states.states = SLEEP;
  }
}

void loop() {
  switch (states.states) {
    case SLEEP:
    //Transition from Sleep to Awake is handled by the AVTransmission function.
    delay(10);
      break;
    case AWAKE:
      if (awakeCounter == 0) {
        eepromWriteState();
        initializeBarometerNormal();
        mpu.enableSleep(false);
        initGroundData();
        awakeCounter++;
      }
      sensors_event_t a;
      mpu.getAccelerometerSensor()->getEvent(&a);
      if (awakeCounter == 1) {
        if (a.acceleration.z >= 20) {
          states.states = FLY;
        }
      }
      break;
    case FLY:
      if (flyCounter == 0) {
        eepromWriteState();
        flyCounter++;
      }
      if (barometerHeight() > MinimumHeight) {
        states.minimumHeight = true;
        eepromWritePrimer();
      }
      // Separation Mechanism Signal is handle by the AVTransmission function.
      //Mach Delay Implementation in milliseconds - Mach Delay Incompatible With EEPROM in current iteration
      if (flyCounter == 0) {
        machDelayA = millis();
      }
      machDelayB = millis();
      machDelayTotal = machDelayB - machDelayA;
      if (machDelayTotal >= machDelay) {
        states.states = READY;
      }
      break;
    case READY:
      if (readyCounter == 0) {
        eepromWriteState();
        readyCounter++;
      }
      secondEvent();
      break;
    case TRIGGERED:
      if (triggeredCounter == 0) {
        eepromWriteState();
        readyCounter++;
      }
      break;
  

  }
  
}
void AVTransmission (int numBytes) {
  if (Wire.available()) {
    byte receivedMessage = Wire.read();
  }
  if (receivedMessage == 0x00) {
    states.states = AWAKE;
  }
  if (receivedMessage == 0x01) {
    states.states = READY;
  }
}
