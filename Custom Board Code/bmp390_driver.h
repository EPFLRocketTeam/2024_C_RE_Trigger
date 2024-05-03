/*
 * bmp390_driver.h
 *
 *  Created on: May 2, 2024
 *      Author: psanc
 */

#include "stm32f3xx.h"
#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <stddef.h>
#include "fatfs.h"
#include "eeprom_driver.h"

#ifndef INC_BMP390_DRIVER_H_
#define INC_BMP390_DRIVER_H_


/*
 * Addresses
 */

//I2C Address
#define BMP3_ADDR_I2C UINT8_C(0x76 << 1) // If SDO connected to GND: UINT8_C(0x76 << 1)If SDO connected to VDDIO UINT8_C(0x77 << 1)

#define BMP390_CHIP_ID 0x00
#define BMP390_REV_ID 0x01
#define BMP390_ERR_REG 0x02
#define BMP390_STATUS 0x03

//Data addresses
#define BMP390_DATA_0 0x04
#define BMP390_DATA_1 0x05
#define BMP390_DATA_2 0x06
#define BMP390_DATA_3 0x07
#define BMP390_DATA_4 0x08
#define BMP390_DATA_5 0x09

#define BMP390_SENSORTIME_0 0x0C
#define BMP390_SENSORTIME_1 0x0D
#define BMP390_SENSORTIME_2 0x0E
#define BMP390_EVENT 0x10
#define BMP390_INT_STATUS 0x11

//FIFO Defines
#define BMP390_FIFO_LENGTH_0 0x12
#define BMP390_FIFO_LENGTH_1 0x13
#define BMP390_FIFO_DATA 0x14
#define BMP390_FIFO_WTM_0 0x15
#define BMP390_FIFO_WTM_1 0x16
#define BMP390_FIFO_CONFIG_1 0x17
#define BMP390_FIFO_CONFIG_2 0x18

#define BMP390_INT_CTRL 0x19
#define BMP390_IF_CONF 0x1A
#define BMP390_PWR_CTRL 0x1B
#define BMP390_OSR 0x1C
#define BMP390_ODR 0x1D
#define BMP390_CONFIG 0x1F
#define BMP390_CMD 0x7E






typedef struct
{
	I2C_HandleTypeDef *i2cHandle;
	uint8_t i2c_address;
	uint32_t pressure;
	float temperature;
	GPIO_TypeDef *interrupt_port;
	uint16_t interrupt_pin;
	uint32_t ground_pressure;
	int32_t ground_altitude;
	float ground_temperature;
	uint16_t eeprom_addr_gnd_pres;
	uint16_t eeprom_addr_gnd_temp;
	uint16_t eeprom_addr_gnd_alt;

}BMP390;

typedef struct
{
    /*! Trim Variables */
    float par_t1;
    float par_t2;
    float par_t3;
    float  par_p1;
    float  par_p2;
    float par_p3;
    float  par_p4;
    float par_p5;
    float par_p6;
    float par_p7;
    float  par_p8;
    float  par_p9;
    float  par_p10;
    float  par_p11;
    float  t_lin;

}BMP3_CALIB_DATA;

#endif /* INC_BMP390_DRIVER_H_ */
