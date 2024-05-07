/*
 * bmp390_driver.c
 *
 *  Created on: May 2, 2024
 *      Author: psanc
 */
void BMP3_initialisation(BMP3 *barometer, EEPROM *eeprom, uint8_t i2c_address, I2C_HandleTypeDef *i2cHandle, BMP3_CALIB_DATA *calib_baro1) {
	barometer -> i2cHandle = i2cHandle;
	barometer -> i2c_address = i2c_address;
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_IsDeviceReady(barometer -> i2cHandle, barometer -> i2c_address, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		printf("FAILURE TO CONNECT TO BMP388\n");
	}
	BMP3_trimming_coeff(barometer, calib_baro);
	BMP3_interrupt_setup(barometer);
	BMP3_setup_normal_mode(barometer);
	BMP3_interrupt_port_setup(barometer);
	get_baro_data_eeprom(barometer, eeprom);
	printf("BMP388 initialisation over\n");
}

HAL_StatusTypeDef BMP3_write_register(BMP3 *barometer, uint16_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Write(barometer -> i2cHandle, barometer -> i2c_address, reg, I2C_MEMADD_SIZE_8BIT, data, BMP3_SIZE ,100);
}

HAL_StatusTypeDef BMP3_read_register(BMP3 *barometer, uint16_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Read(barometer -> i2cHandle, barometer -> i2c_address, reg, I2C_MEMADD_SIZE_8BIT, data, BMP3_SIZE ,100);
}

HAL_StatusTypeDef BMP3_read_registers(BMP3 *barometer, uint16_t reg, uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Read(barometer -> i2cHandle, barometer -> i2c_address, reg, I2C_MEMADD_SIZE_8BIT, data, length , 100);
}


void BMP3_get_temperature(BMP3 *barometer,BMP3_CALIB_DATA *calib_baro)
{
	
}

void BMP3_get_pressure(BMP3 *barometer,BMP3_CALIB_DATA *calib_baro){
}

void BMP3_compensate_temperature(BMP3 *barometer ,BMP3_CALIB_DATA *calib_baro1,
								 int32_t raw_temperature){
}

void BMP3_compensate_pressure(BMP3 *barometer ,BMP3_CALIB_DATA *calib_baro,
							  uint32_t raw_pressure){
}

void BMP3_datalog(BMP3 *barometer, FIL *fil, FRESULT fres, char *sd_buffer,
				  uint8_t* buffer_index, char file_name[]){
}

void BMP3_interrupt_port_setup(BMP3 *barometer){
}

void BMP3_interrupt_setup(BMP3 *barometer){
}

void BMP3_trimming_coeff(BMP3 *barometer, BMP3_CALIB_DATA *calib_baro){
}

void BMP3_setup_normal_mode(BMP3 *barometer){
}

