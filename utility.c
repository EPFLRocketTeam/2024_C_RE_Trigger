/*
* utility.c
*
* Created on: 15 May 2023
* Author: Maxime Leriche
*
*file containing the higher level and more general functions
*/
#include"utility.h"
#include "stm32f3xx.h"
#include "stdio.h"
#include "eeprom_driver.h"
#include "stm32f3xx_ll_gpio.h"
#include "math.h"
#include "bmi160_driver.h"
#include "main.h"
#include <stdlib.h>

float const COEFF_BARO1 =(SEE_LEVEL_TEMP / TEMPERATURE_LAPSE_RATE);
float const COEFF_BARO2 =((UNIVERSAL_GAS_CST * TEMPERATURE_LAPSE_RATE)/
(G * AIR_MOLAR_MASS));

/*collect and store data related to ground altitude in eeprom*/
void init_ground_data(BMP3 *barometer, EEPROM *eeprom)
 {
 barometer -> ground_pressure = barometer -> pressure;
 barometer -> ground_altitude = barometric_altitude(barometer);
 barometer -> ground_temperature = barometer -> temperature;

  #if _DEBUG_MODE >= LIGHT_DEBUG
   printf("ground pressure %lu\n", barometer -> ground_pressure);
   printf("ground altitude %ld \n",barometer -> ground_altitude);
   printf("ground temperature %f\n", barometer -> temperature);
  #endif

 HAL_StatusTypeDef ret;
 ret = EEPROM_write_registers(eeprom, barometer -> eeprom_addr_gnd_pres,
 &barometer -> ground_pressure, 4);

 if(ret != HAL_OK){
   printf("failure to write eeprom press\n");
 }

 HAL_Delay(5); //max write delay in datasheet

 ret = EEPROM_write_registers(eeprom, barometer -> eeprom_addr_gnd_alt,
 &barometer -> ground_altitude, 4);

 if(ret != HAL_OK) {
   printf("failure to write eeprom ground altitude\n");
 }
 /*
 ret = EEPROM_write_registers(eeprom, barometer -> eeprom_addr_gnd_temp,
 &barometer -> ground_temperature,4);
 if(ret != HAL_OK)
 printf("failure to write eeprom temp\n");
 */
 }
/*retrieve previous ground information in case of reboot in flight */
void get_baro_data_eeprom(BMP3 *barometer, EEPROM *eeprom) {
  uint8_t data1[4] = {0};
  HAL_StatusTypeDef ret;
  uint8_t data2[4] = {0};
  if(barometer -> i2cHandle -> Instance == I2C1) {
   ret = EEPROM_read_registers(eeprom, barometer -> eeprom_addr_gnd_pres,
   data1, 4);
   if (ret != HAL_OK) {
     printf("get eeprom ground press error 1 %u", ret); 
   }
     barometer -> ground_pressure = *(uint32_t*)data1;
     ret = EEPROM_read_registers(eeprom, barometer -> eeprom_addr_gnd_alt, data2, 4);
   if (ret != HAL_OK){
     printf("get eeprom ground alt error 1 %u", ret);
   }
 barometer -> ground_altitude = *(int32_t*)data2;

  #if _DEBUG_MODE >= LIGHT_DEBUG
   printf("i2c1 ground press from eeprom %lu \n",
   barometer -> ground_pressure);
  
   printf("i2c1 ground alt from eeprom %lu \n",
   barometer -> ground_altitude);
  #endif
 }
  if(barometer -> i2cHandle -> Instance == I2C2) {
   ret = EEPROM_read_registers(eeprom, barometer -> eeprom_addr_gnd_pres,
   data1, 4);
   if (ret != HAL_OK){
     printf("get eeprom ground press error 2 ");
   }
   barometer -> ground_pressure = *(uint32_t*)data1;
   ret = EEPROM_read_registers(eeprom, barometer -> eeprom_addr_gnd_alt , data2, 4);
   if (ret != HAL_OK){
     printf("get eeprom ground alt error 2 %u", ret);
   }
 barometer -> ground_altitude = *(int32_t*)data2;
   #if _DEBUG_MODE >= LIGHT_DEBUG
   printf("i2c2 ground press from eeprom %lu \n",
   barometer -> ground_pressure);
  
   printf("i2c2 ground alt from eeprom %lu \n",
   barometer -> ground_altitude);
   #endif
 }
}
/* state stored to eeprom*/
 void set_state_eeprom(STATE_MACHINE *states, EEPROM *eeprom) {
   HAL_StatusTypeDef ret;
   HAL_Delay(5);
   ret = EEPROM_write_register(eeprom, EEPROM_STATE_ADDR, &states -> state);
   if(ret != HAL_OK){
     printf("failure to write eeprom state\n");
   }
 } 

 /*state retrieved from eeprom in case of reboot in flight*/
 void get_state_eeprom(STATE_MACHINE *states, EEPROM *eeprom) {
   uint8_t data;
   HAL_StatusTypeDef ret;
   ret = EEPROM_read_register(eeprom, EEPROM_STATE_ADDR, &data);
   if(ret != HAL_OK){
     printf("failure to read eeprom state\n");
   }
   states -> state = data;
 }

/*second event height primer*/
void set_second_event_primer_state_eeprom(STATE_MACHINE *states, EEPROM *eeprom) {
 HAL_StatusTypeDef ret;
 HAL_Delay(5);
 ret = EEPROM_write_register(eeprom, EEPROM_SECOND_EVENT_HEIGHT_STATE,
 &states -> second_event_primer_state);
 if(ret != HAL_OK){
   printf("failure to write eeprom state 2 event\n");
 }
}

/*second event height primer*/
void get_second_event_primer_state_eeprom(STATE_MACHINE *states, EEPROM *eeprom) {
 uint8_t data;
 HAL_StatusTypeDef ret;
 ret = EEPROM_read_register(eeprom, EEPROM_SECOND_EVENT_HEIGHT_STATE, &data);
 if(ret != HAL_OK){
   printf("failure to read eeprom state 2 event\n");
 }
 states -> second_event_primer_state = data;
 }

 /* https://en.wikipedia.org/wiki/Barometric_formula
 * altitude = height from see level*/
 int32_t barometric_altitude(BMP3 *barometer) {
   int32_t altitude = COEFF_BARO1 * (1 - pow(((float)barometer -> pressure /
   SEE_LEVEL_PRESS), COEFF_BARO2));
   return altitude;
 }
 /*height from the ground at launch site*/
 int32_t barometric_height(BMP3 *barometer) {
   int32_t height = (barometric_altitude(barometer) -
   barometer -> ground_altitude);
 return height;
 }
/*create buffer with flight data in order to compute an average altitude rate*/
void temporal_flight_data_compiler(BMP3 *barometer, FLIGHT_DATA *flight_data, uint8_t *counter) {
 uint16_t alt_rate = 0; //m*s^-1
 uint16_t alt_rate_sum = 0; //m*s^-1
 uint16_t alt_rate_avg = 0; //m*s^-1
 if(*counter < FLIGHT_DATA_BUFFER_SIZE -1){
   *counter += 1;
 } else {
   *counter = 0;
 }
 flight_data[*counter].height = barometric_height(barometer);
 flight_data[*counter].time = HAL_GetTick(); //ms
 for(uint8_t i = 0; i < FLIGHT_DATA_BUFFER_SIZE - 2 ; i++) {
   uint16_t delta_t = abs(flight_data[i].time - flight_data[i+1].time);
   uint16_t delta_h = abs(flight_data[i].height - flight_data[i+1].height);
   if(delta_t == 0) //for the first measurement and safety
   {
     printf("delta_t 0 error \n");
     delta_t = 1000; //thread sleep time
   }
   alt_rate = (uint16_t)(((float)delta_h/delta_t) * 1000);
   alt_rate_sum += alt_rate;
 }
  alt_rate_avg = alt_rate_sum / FLIGHT_DATA_BUFFER_SIZE;
  flight_data[*counter].height_rate = alt_rate_avg;
}
/*condition to trigger and stop the current impulse*/
void reefing_second_event(FLIGHT_DATA *flight_data, STATE_MACHINE *states, uint8_t *counter) {
	 if((states -> second_event_primer_state) & (states -> state == FLY))
	 {
		 if((flight_data[*counter].height < SEC_EV_MAX_HGT ) &
				(flight_data[*counter].height > SEC_EV_MIN_HGT ))
		 {
			 if ((flight_data[*counter].height_rate < SEC_EV_MAX_HGT_RATE ) &
					 (flight_data[*counter].height_rate > SEC_EV_MIN_HGT_RATE))
			 {
				 set_second_event_current_impulse();
				 printf("second event \n");
			 } else {
				 reset_second_event_current_impulse();}
		 } else {
			 reset_second_event_current_impulse();}
 } else {
	 reset_second_event_current_impulse();}
}

 /*safety to prime the second event only when certain altitude is reached to
 * avoid unexpected behaviour at launch*/
 void second_event_height_primer_state(STATE_MACHINE *states, FLIGHT_DATA *flight_data, EEPROM *eeprom, uint8_t *counter) {
   if((!(states -> second_event_primer_state)) & (flight_data[*counter].height
  		 > SEC_EV_HGT_PRIMER) & (states -> state == FLY))
   {
	 states -> second_event_primer_state = SET ;
	 set_second_event_primer_state_eeprom(states, eeprom);

	 #if _DEBUG_MODE >= LIGHT_DEBUG
	   printf("second reefing event state set \n");
	 #endif
   }
 }
void set_second_event_current_impulse() {
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
 }
void reset_second_event_current_impulse() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

void GREEN_LED_toggle() {
 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
}

void GREEN_LED_on() {
 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

void GREEN_LED_off() {
 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}
 
void RED_LED_toggle() {
 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
}

void RED_LED_on() {
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
}

void RED_LED_off() {
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
}
