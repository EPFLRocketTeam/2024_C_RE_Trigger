/*
 * bmp390_driver.c
 *
 *  Created on: May 2, 2024
 *      Author: psanc
 */
void BMP3_initialisation(BMP3 *barometer, EEPROM *eeprom, uint8_t i2c_address,
						 I2C_HandleTypeDef *i2cHandle,
						 BMP3_CALIB_DATA *calib_baro1) {
}

HAL_StatusTypeDef BMP3_write_register(BMP3 *barometer, uint16_t reg,
									  uint8_t *data){
}

HAL_StatusTypeDef BMP3_read_register(BMP3 *barometer, uint16_t reg,
									 uint8_t *data){
}

HAL_StatusTypeDef BMP3_read_registers(BMP3 *barometer, uint16_t reg,
									  uint8_t *data, uint8_t length){
}

void BMP3_get_temperature(BMP3 *barometer,BMP3_CALIB_DATA *calib_baro){
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

