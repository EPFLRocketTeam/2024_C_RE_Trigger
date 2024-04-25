
// Includes
#include "Adafruit_BMP280.h"
#include "utility.h"



float const Barometer_Coefficient1 = (REFERENCE_TEMP/TEMPERATURE_LAPSE_RATE)
float const Barometer_Coefficient2 = (UNIVERSAL_GAS_CST * TEMPERATURE_LAPSE_RATE)/(GRAVITY * AIR_MOLAR_MASS)

typedef struct {
    uint8_t state;
    uint8_t minimumHeight;
} STATE_MACHINE;

STATE_MACHINE states;
FLIGHT_DATA flight_data;
GroundData ground_data;


bool MachDelay() {

}

void initGroundData(Adafruit_BMP280 bmp, GroundData ground_data) {
    // Read pressure, temperature, and altitude from the BMP280 sensor
    float pressure = bmp.readPressure();        // Pressure in Pascals
    float temperature = bmp.readTemperature();  // Temperature in degrees Celsius
    float altitude = bmp.readAltitude();        // Altitude in meters

    // Update the ground_data structure
    ground_data.pressure = pressure;
    ground_data.temperature = temperature;
    ground_data.altitude = altitude;
}

int32_t barometerAltitude (Adafruit_BMP280 bmp) {
    int32_t altitude = Barometer_Coefficient1 * (1-pow((bmp.readPressure()/REFERENCE_PRESSURE),Barometer_Coefficient2));
    return altitude;
}

int32_t barometerHeight (Adafruit_BMP280 bmp, GroundData ground_data) {
    int32_t height = barometerAltitude() - ground_data.ground_altitude;
    return height; 
}

void set_second_event_current_impulse() {
    digitalWrite(12, HIGH);
}


// These functions are not used - they are a soeed cakcykatir
void avgAltitudeRate(FLIGHT_DATA *flight_data, uint8_t *counter) {
    uint16_t alt_rate = 0; //m*s^-1
    uint16_t alt_rate_sum = 0; //m*s^-1
    uint16_t alt_rate_avg = 0; //m*s^-1

    if(*counter < FLIGHT_DATA_BUFFER_SIZE -1) {
        *counter += 1;
    } else {
        *counter = 0;
    }

    flight_data[*counter].height = barometerHeight(bmp, ground_data);
    flight_data[*counter].time = HAL_GetTick(); //ms

    for(uint8_t i = 0; i < FLIGHT_DATA_BUFFER_SIZE - 2 ; i++) {
        uint16_t delta_t = abs(flight_data[i].time - flight_data[i+1].time);
        uint16_t delta_h = abs(flight_data[i].height - flight_data[i+1].height);

    if(delta_t == 0) { //for the first measurement and safety
        printf("delta_t 0 error \n");
    delta_t = 1000; //thread sleep time
    }

    alt_rate = (uint16_t)(((float)delta_h/delta_t) * 1000);
    alt_rate_sum += alt_rate;
    }

    alt_rate_avg = alt_rate_sum / FLIGHT_DATA_BUFFER_SIZE;
    flight_data[*counter].height_rate = alt_rate_avg;
}

void secondEventTrigger(FLIGHT_DATA flight_data, STATE_MACHINE states, uint8_t *counter) {
    if (states.state == FLY) {
        if ((flight_data[*counter].height < SEC_EV_MAX_HGT ) & (flight_data[*counter].height > SEC_EV_MIN_HGT )) {
            if ((flight_data[*counter].height_rate < SEC_EV_MAX_HGT_RATE) & (flight_data[*counter].height_rate > SEC_EV_MIN_HGT_RATE) ) {
                set_second_event_current_impulse();
            } else {
                reset_second_event_current_impulse();
            }
        } else {
            reset_second_event_current_impulse();
        }   
    } else {
        reset_second_event_current_impulse();
    }
}



void reset_second_event_current_impulse() {
    digitalWrite(12, LOW);
}
