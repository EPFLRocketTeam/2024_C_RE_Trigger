//Necessary Barometer Values
//Conversion Values
#define REFERENCE_PRESSURE 101325
#define REFERENCE_TEMP (273.15 + 15)
#define TEMPERATURE_LAPSE_RATE 0.0065
#define UNIVERSAL_GAS_CST 8.3143
#define GRAVITY 9.8066
#define AIR_MOLAR_MASS 0.02896

float const Barometer_Coefficient1 = (REFERENCE_TEMP/TEMPERATURE_LAPSE_RATE);
float const Barometer_Coefficient2 = (UNIVERSAL_GAS_CST * TEMPERATURE_LAPSE_RATE)/(GRAVITY * AIR_MOLAR_MASS);

// Extra
#define FLIGHT_DATA_BUFFER_SIZE 5 //to compute average height rate

// Flight Values
#define secondEventHeightPrimer 4000 // SECOND EVENT HEIGHT PRIMER
#define secondEventMaxHeight 450 // SECOND EVENT MAX HEIGHT
#define secondEventMinHeight 100 // SECOND EVENT MIN HEIGHT
#define secondEventMaxSpeed 40 // SECOND EVENT MAX HEIGHT RATE
#define secondEventMinSpeed 15 // SECOND EVENT MIN HEIGHT RATE



void initializeBarometerSleep() {
  Serial.begin(9600);
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_SLEEP,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void initializeBarometerNormal() {
  Serial.begin(9600);
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
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

void printBarometerValues() {
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");

    Serial.println();
    delay(2000);
}


//Initalize Ground Data
void initGroundData() {
  // Read pressure, temperature, and altitude from the BMP280 sensor
  float pressure = bmp.readPressure();        // Pressure in Pascals
  float temperature = bmp.readTemperature();  // Temperature in degrees Celsius
  float altitude = bmp.readAltitude();        // Altitude in meters

  // Update the ground_data structure
  groundData.groundPressure = pressure;
  groundData.groundTemperature = temperature;
  groundData.groundAltitude = altitude;
}

// Altitude Converter
float barometerAltitude () {
    float altitude = Barometer_Coefficient1 * (1-pow((bmp.readPressure()/REFERENCE_PRESSURE),Barometer_Coefficient2));
    return altitude;
}

// Height Converter
float barometerHeight () {
    float height = bmp.readAltitude(1013.25) - groundData.groundAltitude;
    return height; 
}

void averageSpeed() {
  float alt_rate = 0;
  float alt_rate_sum = 0;
  float alt_rate_avg = 0;
  if (counter < FLIGHT_DATA_BUFFER_SIZE -1) {
    counter += 1;
  } else {
    counter = 0;
  }
  flightDataArray[counter].height = barometerHeight();
  flightDataArray[counter].time = millis(); 

  for(uint8_t i = 0; i <FLIGHT_DATA_BUFFER_SIZE -1 ; i++) {
    uint16_t delta_t = abs(flightDataArray[i].time-flightDataArray[i+1].time);
    uint16_t delta_h = abs(flightDataArray[i].height-flightDataArray[i+1].height);
    if(delta_t == 0) //for the first measurement and safety
      {
        printf("delta_t 0 error \n");
        delta_t = 1000; //thread sleep time
      }   
    alt_rate = (uint16_t)(((float)delta_h/delta_t) * 1000);
    alt_rate_sum += alt_rate;
 }

 alt_rate_avg = alt_rate_sum / FLIGHT_DATA_BUFFER_SIZE;
 flightDataArray[counter].heightRate = alt_rate_avg;
}

