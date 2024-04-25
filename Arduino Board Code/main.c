
#include "stdio.h"
#include <stdint.h>
#include <string.h>
#include <stdarg.h>

// Private Functions
#include "main.h"
#include <Checksettings.c>
#include <utility.c>
#include <utility.h>

// Avionic Signals
#define AVseparationSignalPort 10;
#define AValtimeterSignalPort 11;

// Sensor initliazations
#include <MPU6050.h> // Driver
#include <Adafruit_BMP280.h> // Driver
#include <Wire.h> // Communication I2C
Adafruit_BMP280 bmp;
MPU6050 mpu;

//Other Constants (not in utility)
#define LaunchAcceleration 20; 
#define MinimumHeight 100;


// Mach Delay functions
unsigned long flightStartTime = 0;
unsigned long currentTime = 0;
const long waitDuration = 100000; // 100 seconds in milliseconds


void setup() 
{
    // Accelerometer
    Serial.begin(9600);

    Serial.println("Initialize MPU6050");

    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    {
        Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        delay(500);
    }

  // Accelerometer offsets
  // mpu.setAccelOffsetX();
  // mpu.setAccelOffsetY();
  // mpu.setAccelOffsetZ();
    checkSettings();
    delay(500);


    // Barometer
    Serial.println(F("BMP280 test"));
    unsigned status;
    //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
    status = bmp.begin();
    if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or " "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
    }

      /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

}

void loop() {
    states.state = ON;
    states.minimumHeight = LOW;
    switch (states.state) {
        case ON:
            Serial.print("Waiting for launch");
                initGroundData(bmp, ground_data)
            if (normAccel.YAxis > LaunchAcceleration) {
                states.state = FLY;
                flightStartTime = millis();
            }
            break;
        case FLY:
            Serial.print("In flight")
            currentTime = millis();
            // Set Minimum Height
            if (barometerHeight(bmp) > MinimumHeight) {
                states.minimumHeight = REACHED;
            }
            //State Switch
            if (currentTime - flightStartTime == waitDuration) {
                states.state = Ready;
            }
            if (digitalRead(SeparationMechPort) == HIGH) {
                states.state = Ready;
            }
            break;
        case READY:
            if (digitalRead(AValtimeterSignalPort) == HIGH) {
                set_second_event_current_impulse();
            }
            if (barometerHeight(bmp) < 400) {
                if (states.minimumHeight == REACHED) {
                    set_second_event_current_impulse();
                }
            }
    }
}