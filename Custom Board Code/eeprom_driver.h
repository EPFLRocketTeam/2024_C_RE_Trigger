/*
 * eeprom_driver.h
 *
 *  Created on: May 7, 2024
 *      Author: psanc
 */

#ifndef INC_EEPROM_DRIVER_H_
#define INC_EEPROM_DRIVER_H_

#include "stm32f3xx.h"

typedef struct
{
	I2C_HandleTypeDef *i2cHandle;
	uint8_t i2c_address;

}EEPROM;

#define EEPROM_I2C_ADDR UINT8_C(0x50 << 1)
#define EEPROM_PAGE_SIZE UINT8_C(8)
#define EEPROM_NB_PAGE UINT8_C(128)
#define EEPROM_SIZE UINT8_C(0x01)
#define EEPROM_PAGE_1_ADDR UINT8_C(0X0)

//ADDRESS OF DATA
#define EEPROM_STATE_ADDR  1
#define EEPROM_SECOND_EVENT_HEIGHT_STATE 2
#define EEPROM_IMPULSE_COUNTER_ADDR

#define EEPROM_GROUND_PRESS_ADDR_1  3
#define EEPROM_GROUND_TEMP_ADDR_1  8
#define EEPROM_GROUND_ALT_ADDR_1 12

#define EEPROM_GROUND_PRESS_ADDR_2  16
#define EEPROM_GROUND_TEMP_ADDR_2  20
#define EEPROM_GROUND_ALT_ADDR_2 24

#define EEPROM_IWDG_COUNTER_ADDR  64
#define EEPROM_POWER_LOSS_COUNTER_ADDR  65

void EEPROM_init(EEPROM *eeprom, uint8_t i2c_address,
								I2C_HandleTypeDef *i2cHandle);

HAL_StatusTypeDef EEPROM_write_register(EEPROM *eeprom, uint16_t reg,
										uint8_t *data);

HAL_StatusTypeDef EEPROM_write_registers(EEPROM *eeprom, uint16_t reg,
										 void *data, size_t data_length);

HAL_StatusTypeDef EEPROM_read_register(EEPROM *eeprom, uint16_t reg,
									   uint8_t *data);

HAL_StatusTypeDef EEPROM_read_registers(EEPROM *eeprom, uint16_t reg,
										uint8_t *data, uint8_t length);

#endif /* INC_EEPROM_DRIVER_H_ */




