/*
 * bmp390_driver.h
 *
 *  Created on: May 2, 2024
 *      Author: psanc
 */


#include <eeprom_driver.h>
#include "stdint.h"
#include "stdio.h"
#include "string.h"

HAL_StatusTypeDef EEPROM_write_register(EEPROM *eeprom, uint16_t reg,
										uint8_t *data)
{
	return HAL_I2C_Mem_Write(eeprom -> i2cHandle, eeprom -> i2c_address, reg,
			I2C_MEMADD_SIZE_8BIT, data, EEPROM_SIZE ,1000);
}

HAL_StatusTypeDef EEPROM_write_registers(EEPROM *eeprom, uint16_t reg,
										 void *data, size_t data_length)
{
	uint8_t buffer[data_length];
	memcpy(buffer, data, data_length);

	return HAL_I2C_Mem_Write(eeprom -> i2cHandle, eeprom -> i2c_address, reg,
			I2C_MEMADD_SIZE_8BIT, buffer, data_length ,1000);
}

HAL_StatusTypeDef EEPROM_read_register(EEPROM *eeprom, uint16_t reg,
									   uint8_t *data)
{
	return HAL_I2C_Mem_Read(eeprom -> i2cHandle, eeprom -> i2c_address, reg,
			I2C_MEMADD_SIZE_8BIT, data, EEPROM_SIZE ,1000);
}

HAL_StatusTypeDef EEPROM_read_registers(EEPROM *eeprom, uint16_t reg,
										uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Read(eeprom -> i2cHandle, eeprom -> i2c_address, reg,
			I2C_MEMADD_SIZE_8BIT, data, length , 1000);
}

void EEPROM_init(EEPROM *eeprom, uint8_t i2c_address,
				 I2C_HandleTypeDef *i2cHandle)
{
	eeprom -> i2cHandle = i2cHandle;
	eeprom-> i2c_address = i2c_address;

	HAL_StatusTypeDef ret;

	ret = HAL_I2C_IsDeviceReady(eeprom -> i2cHandle, eeprom -> i2c_address,
								1, 1000);
	if (ret == HAL_OK){
		printf("connection to eeprom successful\n");}

}
