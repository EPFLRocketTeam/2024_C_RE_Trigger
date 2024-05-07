/*
 * bmp390_driver.c
 *
 *  Created on: May 2, 2024
 *      Author: psanc
 */
void BMP390_initialisation(BMP3 *barometer, EEPROM *eeprom, uint8_t i2c_address, I2C_HandleTypeDef *i2cHandle, BMP3_CALIB_DATA *calib_baro1) {
	barometer -> i2cHandle = i2cHandle;
	barometer -> i2c_address = i2c_address;
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_IsDeviceReady(barometer -> i2cHandle, barometer -> i2c_address, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		printf("FAILURE TO CONNECT TO BMP390\n");
	}
	BMP390_trimming_coeff(barometer, calib_baro);
	BMP390_interrupt_setup(barometer);
	BMP390_setup_normal_mode(barometer);
	BMP390_interrupt_port_setup(barometer);
	get_baro_data_eeprom(barometer, eeprom);
	printf("BMP390 initialisation over\n");
}

HAL_StatusTypeDef BMP390_write_register(BMP3 *barometer, uint16_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Write(barometer -> i2cHandle, barometer -> i2c_address, reg, I2C_MEMADD_SIZE_8BIT, data, BMP3_SIZE ,100);
}

HAL_StatusTypeDef BMP390_read_register(BMP3 *barometer, uint16_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Read(barometer -> i2cHandle, barometer -> i2c_address, reg, I2C_MEMADD_SIZE_8BIT, data, BMP3_SIZE ,100);
}

HAL_StatusTypeDef BMP390_read_registers(BMP3 *barometer, uint16_t reg, uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Read(barometer -> i2cHandle, barometer -> i2c_address, reg, I2C_MEMADD_SIZE_8BIT, data, length , 100);
}


void BMP390_get_temperature(BMP3 *barometer,BMP3_CALIB_DATA *calib_baro)
{
	uint8_t data[3] = {0};
	HAL_StatusTypeDef ret;
	ret = BMP3_read_registers(barometer, BMP390_DATA_3, data , 3);
        if(ret != HAL_OK){
		printf("failure to read temp bmp390\n");
	}
	int32_t raw_temperature = (((uint32_t)data[2] << 16) | (uint32_t)data[1] << 8 | (uint32_t)data[0] );
	BMP3_compensate_temperature(barometer ,calib_baro, raw_temperature);
	#if _DEBUG_MODE >= FULL_DEBUG
   		printf("temperature %f \n", barometer -> temperature);
	#endif
}	


void BMP3_get_pressure(BMP3 *barometer,BMP3_CALIB_DATA *calib_baro){
	uint8_t data[3] = {0};
	HAL_StatusTypeDef ret;
	//temperature needed to compute pressure
	BMP390_get_temperature(barometer, calib_baro);
	ret = BMP3_read_registers(barometer, BMP390_DATA_0, data, 3);
	if(ret != HAL_OK){
		printf("failure to read pressure bmp390\n");
	}
	uint32_t raw_pressure = (((uint32_t)data[2] << 16) | (uint32_t)data[1] << 8 | (uint32_t)data[0] );
	BMP3_compensate_pressure(barometer, calib_baro, raw_pressure);
	#if _DEBUG_MODE >= FULL_DEBUG
		printf("pressure %lu \n", barometer -> pressure);
	#endif
}

void BMP3_compensate_temperature(BMP3 *barometer ,BMP3_CALIB_DATA *calib_baro1, int32_t raw_temperature)
{
	// Page 55 of datasheet, edited to be a void function that modifies the BMP structure instead of static return
	float t_lin;
	float partial_data1;
	float partial_data2;
	partial_data1 = (float)((raw_temperature) - (calib_baro -> par_t1));
	partial_data2 = (float)(partial_data1 * calib_baro -> par_t2);
	/* Update the compensated temperature in calib structure since this is
	* needed for pressure calculation */
	t_lin = partial_data2 + (partial_data1 * partial_data1) *
			calib_baro -> par_t3;

	barometer -> temperature = t_lin ;
	calib_baro -> t_lin  = t_lin ;
}

void BMP3_compensate_pressure(BMP3 *barometer ,BMP3_CALIB_DATA *calib_baro, uint32_t raw_pressure)
{
	//Page 56 of datasheet, edited to be a void function that modifies the BMP structure instead of a static return
	/* Variable to store the compensated pressure */
	float comp_press;
	/* Temporary variables used for compensation */
	float partial_data1;
	float partial_data2;
	float partial_data3;
	float partial_data4;
	float partial_out1;
	float partial_out2;
	/* Calibration data */

	partial_data1 = calib_baro -> par_p6 * calib_baro -> t_lin;
	partial_data2 = calib_baro -> par_p7 * (calib_baro -> t_lin *
					calib_baro -> t_lin);

	partial_data3 = calib_baro -> par_p8 * (calib_baro -> t_lin *
					calib_baro -> t_lin * calib_baro -> t_lin);

	partial_out1 = calib_baro -> par_p5 + partial_data1 + partial_data2 +
				   partial_data3;

	partial_data1 = calib_baro -> par_p2 * calib_baro -> t_lin;
	partial_data2 = calib_baro -> par_p3 * (calib_baro -> t_lin *
					calib_baro -> t_lin);

	partial_data3 = calib_baro -> par_p4 * (calib_baro -> t_lin *
					calib_baro -> t_lin * calib_baro -> t_lin);

	partial_out2 = (float)raw_pressure *
	(calib_baro -> par_p1 + partial_data1 + partial_data2 + partial_data3);

	partial_data1 = (float)raw_pressure * (float)raw_pressure;
	partial_data2 = calib_baro -> par_p9 + calib_baro -> par_p10 *
					calib_baro -> t_lin;

	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + ((float)raw_pressure * (float)raw_pressure *
			        (float)raw_pressure) * calib_baro -> par_p11;

	comp_press = partial_out1 + partial_out2 + partial_data4;
	barometer -> pressure = (uint32_t)comp_press;
}

/*Data log of the pressure and temperature on the micro-SD card,
 * a buffer is implemented to limit the write occurrence and thus the
 * energy consumption*/
void BMP3_datalog(BMP3 *barometer, FIL *fil, FRESULT fres, char *sd_buffer, uint8_t* buffer_index, char file_name[])
{
	//5 press + space + comma + 6 temp + \r + \n = 15 bytes worst case
	sprintf(sd_buffer + BMP3_SD_DATA_SIZE * (*buffer_index),"%lu, %.2f \r\n", barometer -> pressure, barometer -> temperature);
	*buffer_index += 1;
	if (*buffer_index >= BMP3_SD_BUFFER_CYCLE)
	{
		static UINT bytesWrote = 0;
		 //f_open has a high stack cost!
		fres = f_open(fil, file_name , FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
		if (fres != FR_OK){
			printf("f_open error (%i)\r\n", fres);
		}
		fres = f_lseek(fil, fil -> fsize);
		if (fres != FR_OK){
			printf("f_lseek bmp (%i)\r\n", fres);
		}
		fres = f_write(fil, sd_buffer, BMP3_SD_BUFFER_SIZE, &bytesWrote);
		if (fres != FR_OK){
			printf("f_write error bmp datalog(%i)\r\n", fres);	
		}
	#if _DEBUG_MODE >= LIGHT_DEBUG
		printf("write barometer data to sd \n");
	#endif
	f_close(fil);
	*buffer_index = 0;
	}
}

void BMP3_interrupt_port_setup(BMP3 *barometer){
	// difference if i2c handle is 1 or 2 (different i2c port)
		if(barometer -> i2cHandle -> Instance == I2C1)
	{
		barometer -> interrupt_port = I2C1_INT1_GPIO_Port;
		barometer -> interrupt_pin = I2C1_INT1_Pin;
		HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, 10, 0);
		HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);
		barometer -> eeprom_addr_gnd_pres = EEPROM_GROUND_PRESS_ADDR_1;
		barometer -> eeprom_addr_gnd_temp = EEPROM_GROUND_TEMP_ADDR_1;
		barometer -> eeprom_addr_gnd_alt = EEPROM_GROUND_ALT_ADDR_1;
		printf("BMP388 interrupt port config I2C1\n");
	}

	if (barometer -> i2cHandle -> Instance == I2C2)
	{
		barometer -> interrupt_port = I2C2_INT1_GPIO_Port;
		barometer -> interrupt_pin = I2C2_INT1_Pin;
		barometer -> eeprom_addr_gnd_pres = EEPROM_GROUND_PRESS_ADDR_2;
		barometer -> eeprom_addr_gnd_temp = EEPROM_GROUND_TEMP_ADDR_2;
		barometer -> eeprom_addr_gnd_alt = EEPROM_GROUND_ALT_ADDR_2;
		printf("BMP388 interrupt port config I2C2\n");
	}
}

void BMP3_interrupt_setup(BMP3 *barometer)
{
	// Activating the Data Ready Interrupt for Pressure and Temperature, which will activate by switching from low to high
	HAL_StatusTypeDef ret;
	uint8_t reg_config  =  BMP390_INT_DRDY_EN | BMP390_INT_LEVEL ; // Level is one by default (check header file), thus setting the interrupt pin to active high, as after a reset.

	ret = BMP3_write_register(barometer, BMP390_INT_CTRL, &reg_config);
	if(ret != HAL_OK){
		printf("Error writing register in BMP388_init\n");
	}
}

void BMP3_trimming_coeff(BMP3 *barometer, BMP3_CALIB_DATA *calib_baro)
{
	static uint8_t data = 0;
	static uint8_t data2[2] = {0};
	static HAL_StatusTypeDef ret;

	// Page 55 of Datasheet to find formulas
	// Page 28 of Datasheet for registers
	ret = BMP3_read_registers(barometer, BMP390_PAR_T1_ADR , data2 , 2);
	calib_baro -> par_t1 = (float)(((uint16_t)data2[1] << 8 | data2[0]) / pow(2,-8));

	if(ret != HAL_OK){
		printf("Error in BMP trimming coeff par_t1\n");}

	ret = BMP3_read_registers(barometer, BMP3_PAR_T2_ADR , data2 , 2);
	calib_baro -> par_t2 = (float)(((uint16_t)data2[1] << 8 | data2[0]) /
							pow(2,30));

	if(ret != HAL_OK){
		printf("Error in BMP trimming coeff par_t2\n");}

	ret = BMP3_read_register(barometer, BMP3_PAR_T3_ADR, &data);
	calib_baro -> par_t3 = (float)(((int8_t)data) / pow(2,48));
	if(ret != HAL_OK){
		printf("Error in BMP trimming coeff par_t3\n");}

	ret = BMP3_read_registers(barometer, BMP3_PAR_P1_ADR , data2 , 2);
	calib_baro -> par_p1 = (float)(((int16_t)(data2[1] << 8 | data2[0]) -
							pow(2,14))/ pow(2,20));

	if(ret != HAL_OK){
		printf("Error in BMP trimming coeff par_p1\n");}

	ret = BMP3_read_registers(barometer, BMP3_PAR_P2_ADR , data2 , 2);
	calib_baro -> par_p2 = (float)(((int16_t)(data2[1] << 8 | data2[0]) -
							pow(2,14))/ pow(2,29));

	if(ret != HAL_OK){
		printf("Error in BMP trimming coeff par_p2\n");}

	ret = BMP3_read_register(barometer, BMP3_PAR_P3_ADR, &data);
	calib_baro -> par_p3 = (float)((int8_t)data / pow(2,32));
	if(ret != HAL_OK){
		printf("Error in BMP trimming coeff par_p3\n");}

	ret = BMP3_read_register(barometer, BMP3_PAR_P4_ADR, &data);
	calib_baro -> par_p4 = (float)((int8_t)data / pow(2,37));
	if(ret != HAL_OK){
		printf("Error in BMP trimming coeff par_p4\n");}

	ret = BMP3_read_registers(barometer, BMP3_PAR_P5_ADR , data2 , 2);
	calib_baro -> par_p5 = (float)((uint16_t)(data2[1] << 8 | data2[0]) /
							pow(2,-3));

	if(ret != HAL_OK){
		printf("Error in BMP trimming coeff par_p5\n");}

	ret = BMP3_read_registers(barometer, BMP3_PAR_P6_ADR , data2 , 2);
	calib_baro -> par_p6 = (float)((uint16_t)(data2[1] << 8 | data2[0])/
							pow(2,6));

	if(ret != HAL_OK)
		printf("Error in BMP trimming coeff par_p6\n");

	ret = BMP3_read_register(barometer, BMP3_PAR_P7_ADR, &data);
	calib_baro -> par_p7 = (float)((int8_t)data / pow(2,8));
	if(ret != HAL_OK){
		printf("Error in BMP trimming coeff par_p7\n");}

	ret = BMP3_read_register(barometer, BMP3_PAR_P8_ADR, &data);
	calib_baro -> par_p8 = (float)((int8_t)data / pow(2,15));
	if(ret != HAL_OK){
		printf("Error in BMP trimming coeff par_p8\n");}

	ret = BMP3_read_registers(barometer, BMP3_PAR_P9_ADR , data2 , 2);
	calib_baro -> par_p9 = (float)((int16_t)(data2[1] << 8 | data2[0]) /
							pow(2,48));

	if(ret != HAL_OK){
		printf("Error in BMP trimming coeff par_p9\n");}

	ret = BMP3_read_register(barometer, BMP3_PAR_P10_ADR, &data);
	calib_baro -> par_p10 = (float)((int8_t)data / pow(2,48));
	if(ret != HAL_OK){
		printf("Error in BMP trimming coeff par_p10\n");}

	ret = BMP3_read_register(barometer, BMP3_PAR_P11_ADR, &data);
	calib_baro -> par_p11 = (float)((int8_t)data / pow(2,65));
	if(ret != HAL_OK){
		printf("Error in BMP trimming coeff par_p11\n");}v
}

void BMP390_setup_normal_mode(BMP390 *barometer)
{
	uint8_t BMP390_settings[10] = {0};
	uint16_t BMP390_settings_addr[10] = {0};

	//BMP390_config	Irr filter
	BMP390_settings[0] = BMP3_IIR_FILTER_COEFF_3 << 1; // This is a 3 filter coefficient, shifted by 1 because bits 1-3 count, 0 doesn't count!
	BMP390_settings_addr[0] = BMP390_CONFIG;

	//BMP390_odr sampling speed
	BMP390_settings[1] = BMP390_ODR_0_78_HZ;
	BMP390_settings_addr[1] = BMP390_ODR;

	//BMP390_osr
	BMP390_settings[2] = BMP390_OVERSAMPLING_8X | BMP390_OVERSAMPLING_2X << 3; //Shifting the bits for the  temperature oversampling
	BMP390_settings_addr[2] = BMP390_OSR;

	//BMP390_pwr_ctrl
	BMP390_settings[3] = BMP390_MODE_NORMAL << 4| BMP390_PWR_PRESS_EN  << 1| BMP390_PWR_PRESS_EN; //Shifting the pressure enable is the same as the temperature enable (simpler to shift a single bit)
	BMP390_settings_addr[3] = BMP390_PWR_CTRL;

	//BMP390_if_conf
	BMP390_settings[4] = 0x00;
	BMP390_settings_addr[4] = BMP390_REG_IF_CONF;

	//BMP390_fifo_config_2
	BMP390_settings[5] = 0x02;
	BMP390_settings_addr[5] = BMP390_REG_FIFO_CONFIG_2; //16x subsampling for FIFO

	//BMP390_fifo_config_1
	BMP390_settings[6] = 0x02;
	BMP390_settings_addr[6] = BMP390_REG_FIFO_CONFIG_1;  //Returns sensor time frame after last valid data frame

	//Watermark interrupts are unused so these initializations are technically useless.
	//BMP390_fifo_wtm_1
	BMP390_settings[7] = 0x00;
	BMP390_settings_addr[7] = 0x15;  // Water

	//BMP390_fifo_wtm_0
	BMP390_settings[8] = 0x00;
	BMP390_settings_addr[8] = 0x16;  //

	HAL_StatusTypeDef ret;
	for (int i = 0; i <= 8 ;i++)
	{
		ret = BMP3_write_register(barometer, BMP390_settings_addr[i],
								  &BMP390_settings[i]);
		if(ret != HAL_OK){
			printf("Error writing register in BMP390_init\n");}
	}
}

void BMP390_setup_sleep_mode(BMP390 *barometer)
{
	uint8_t BMP390_settings[10] = {0};
	uint16_t BMP390_settings_addr[10] = {0};

	//BMP390_config	Irr filter
	BMP390_settings[0] = BMP390_IIR_FILTER_DISABLE << 1; // Disables IIR filter
	BMP390_settings_addr[0] = BMP390_CONFIG;

	//BMP390_odr sampling speed
	BMP390_settings[1] = BMP390_ODR_0_001_HZ; //Lowest ODR possible
	BMP390_settings_addr[1] = BMP390_ODR;

	//BMP390_osr
	BMP390_settings[2] = BMP390_NO_OVERSAMPLING | BMP390_NO_OVERSAMPLING << 3; //Shifting the bits for the  temperature oversampling - Remove all oversampling
	BMP390_settings_addr[2] = BMP390_OSR;

	//BMP390_pwr_ctrl
	BMP390_settings[3] = BMP3_MODE_SLEEP << 4| 0x00  << 1| 0x00; //Disables pressure and temperature measurements
	BMP390_settings_addr[3] = BMP390_PWR_CTRL;

	//BMP390_if_conf
	BMP390_settings[4] = 0x00;
	BMP390_settings_addr[4] = BMP390_REG_IF_CONF;

	//BMP390_fifo_config_2
	BMP390_settings[5] = 0x00;
	BMP390_settings_addr[5] = BMP390_REG_FIFO_CONFIG_2; //2x subsampling for FIFO

	//BMP390_fifo_config_1
	BMP390_settings[6] = 0x02;
	BMP390_settings_addr[6] = BMP390_REG_FIFO_CONFIG_1;  //Returns sensor time frame after last valid data frame

	//Watermark interrupts are unused so these initializations are technically useless.
	//BMP390_fifo_wtm_1
	BMP390_settings[7] = 0x00;
	BMP390_settings_addr[7] = 0x15;  // Water

	//BMP390_fifo_wtm_0
	BMP390_settings[8] = 0x00;
	BMP390_settings_addr[8] = 0x16;  //

	HAL_StatusTypeDef ret;
	for (int i = 0; i <= 8 ;i++)
	{
		ret = BMP3_write_register(barometer, BMP390_settings_addr[i],
								  &BMP390_settings[i]);
		if(ret != HAL_OK){
			printf("Error writing register in BMP390_init\n");}
	}
}

