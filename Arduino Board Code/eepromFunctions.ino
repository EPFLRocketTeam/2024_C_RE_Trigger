#define eepromStateAddress 1
#define eepromPrimerAddress 2
#define eepromGroundPressureAddress 3
#define eepromGroundAltitudeAddress 4
#define eepromGroundTemperatureAddress 5

void initializeEEPROM() {
   if (i2ceeprom.begin(0x50)) {  // you can stick the new i2c addr in here, e.g. begin(0x51);
    Serial.println("Found I2C EEPROM");
  } else {
    Serial.println("I2C EEPROM not identified ... check your connections?\r\n");
    while (1) delay(10);
  }
}
//State Read and Right
void eepromWriteState() {
  i2ceeprom.write(eepromStateAddress, states.states);
}

void eepromReadState() {
  states.states = i2ceeprom.read(eepromStateAddress);
}

//Minimum Height/Primer Read and Write to EEPROM
void eepromWritePrimer() {
  i2ceeprom.write(eepromPrimerAddress, states.minimumHeight);
}

void eepromReadPrimer() {
  states.minimumHeight = i2ceeprom.read(eepromPrimerAddress);
}

//Ground Data Read and Write
void eepromWriteGroundData() {
  i2ceeprom.write(eepromGroundPressureAddress, groundData.groundPressure);
  i2ceeprom.write(eepromGroundTemperatureAddress, groundData.groundTemperature);
  i2ceeprom.write(eepromGroundAltitudeAddress, groundData.groundAltitude);
}

void eepromReadGroundData() {
  groundData.groundPressure = i2ceeprom.read(eepromGroundPressureAddress);
  groundData.groundTemperature = i2ceeprom.read(eepromGroundTemperatureAddress);
  groundData.groundAltitude = i2ceeprom.read(eepromGroundAltitudeAddress);
}
