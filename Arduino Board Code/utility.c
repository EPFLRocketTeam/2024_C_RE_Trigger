
// Includes
#include "Adafruit_BMP280.h"

//Defines
#define REFERENCE_PRESSURE 101325
#define REFERENCE_TEMP (273.15 + 15)
#define TEMPERATURE_LAPSE_RATE 0.0065
#define UNIVERSAL_GAS_CST 8.3143
#define GRAVITY 9.8066
#define AIR_MOLAR_MASS 0.02896

float const Barometer_Coefficient1 = (REFERENCE_TEMP/TEMPERATURE_LAPSE_RATE)
float const Barometer_Coefficient2 = (UNIVERSAL_GAS_CST * TEMPERATURE_LAPSE_RATE)/(GRAVITY * AIR_MOLAR_MASS)

typedef struct {
    float ground_altitude;   // Altitude in meters
    float grounbd_pressure;   // Pressure in Pascals
    float ground_temperature; // Temperature in degrees Celsius
} GroundData;

typedef struct {
    uint32_t time;
    uint16_t height_rate;
    int32_t height;
} FLIGHT_DATA;

GroundData ground_data;

void initGroundData(Adafruit_BMP280 bmp) {
    // Read pressure, temperature, and altitude from the BMP280 sensor
    float pressure = bmp.readPressure();        // Pressure in Pascals
    float temperature = bmp.readTemperature();  // Temperature in degrees Celsius
    float altitude = bmp.readAltitude();        // Altitude in meters

    // Update the ground_data structure
    ground_data.pressure = pressure;
    ground_data.temperature = temperature;
    ground_data.altitude = altitude;
}

int32_t barometerAltitude () {
    int32_t altitude = Barometer_Coefficient1 * (1-pow((bmp.readPressure()/REFERENCE_PRESSURE),Barometer_Coefficient2));
    return altitude;
}

int32_t barometerHeight () {
    int32_t height = barometerAltitude() - ground_data.ground_altitude;
    return height; 
}

void avgAltitudeRate()

void secondEventTrigger() {

}
