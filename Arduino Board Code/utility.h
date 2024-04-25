#ifndef INC_UTILITY_H_
#define INC_UTILITY_H_

#include "bmp_driver.h"
#include "bmi160_driver.h"
#include "eeprom_driver.h"



/*constant to compute the barometric altitude*/
#define REFERENCE_PRESSURE 101325
#define REFERENCE_TEMP (273.15 + 15)
#define TEMPERATURE_LAPSE_RATE 0.0065
#define UNIVERSAL_GAS_CST 8.3143
#define GRAVITY 9.8066
#define AIR_MOLAR_MASS 0.02896

#define FLIGHT_DATA_BUFFER_SIZE 5 //to compute average height rate

#define SEC_EV_HGT_PRIMER 4000 // SECOND EVENT HEIGHT PRIMER
#define SEC_EV_MAX_HGT 450 // SECOND EVENT MAX HEIGHT
#define SEC_EV_MIN_HGT 100 // SECOND EVENT MIN HEIGHT
#define SEC_EV_MAX_HGT_RATE 40 // SECOND EVENT MAX HEIGHT RATE
#define SEC_EV_MIN_HGT_RATE 15 // SECOND EVENT MIN HEIGHT RATE

typedef struct {
    float ground_altitude;   // Altitude in meters
    float grounbd_pressure;   // Pressure in Pascals
    float ground_temperature; // Temperature in degrees Celsius
} GroundData;

typedef struct {
    uint32_t time;
    uint16_t height_rate;
    int32_t height;
}FLIGHT_DATA;

void initGroundData(Adafruit_BMP280 bmp, GroundData ground_data);

int32_t barometerAltitude (Adafruit_BMP280 bmp);

int32_t barometerHeight (Adafruit_BMP280 bmp, GroundData ground_data);

void avgAltitudeRate(FLIGHT_DATA *flight_data, uint8_t *counter);

void secondEventTrigger(FLIGHT_DATA flight_data, STATE_MACHINE states, uint8_t *counter);

void set_second_event_current_impulse();

void reset_second_event_current_impulse();

#endif /* INC_UTILITY_H_ */
