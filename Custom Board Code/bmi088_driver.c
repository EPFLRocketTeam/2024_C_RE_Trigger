/*
 * bmi088_driver.c
 *
 *  Created on: May 1, 2024
 *      Author: psanc
 */
#include "bmi088_driver.h"

void BMI088_Initialise(BMI088 *imu, uint8_t i2c_address, I2C_HandleTypeDef *i2cHandle,
				 uint8_t accel_range, uint16_t gyro_range) {
	// Initialize variables in the imu struct
	imu -> i2cHandle = i2cHandle;
	imu -> i2c_address = i2c_address;
	imu -> accel_range = accel_range;
	imu -> gyro_range = gyro_range;

	//Check if communication is ready
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_IsDeviceReady(imu -> i2cHandle, imu -> i2c_address, 1,
									HAL_MAX_DELAY);
}

/*
 * LOW LEVEL FUNCTIONS
 */

HAL_StatusTypeDef BMI088_readACCLRegister(BMI088 *imu, uint16_t reg,
									   uint8_t *data) {
	return HAL_I2C_MEM_READ (imu->i2cHandle, BMI088_I2C_ACCL_ADDR, reg, I2C_MEMADD_SIZE_8BIT,data, 1, HAL_MAX_DELAY);
}


HAL_StatusTypeDef BMI088_readACCLRegisters(BMI088 *imu, uint16_t reg,
										uint8_t *data, uint8_t length) {
	return HAL_I2C_MEM_READ (imu->i2cHandle, BMI088_I2C_ACCL_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);

}

HAL_StatusTypeDef BMI088_writeACCLRegister(BMI088 *imu, uint16_t reg,
										uint8_t *data) {
	return HAL_I2C_MEM_WRITE (imu->i2cHandle, BMI088_I2C_ACCL_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BMI088_transmitCommandAccl(BMI088 *imu,uint8_t *data) {
	return HAL_I2C_Master_Transmit(imu -> i2cHandle, BMI088_I2C_ACCL_ADDR, data,
									   1 ,HAL_MAX_DELAY);
}


//Gyroscope functions - Not useful to us.
HAL_StatusTypeDef BMI088_readGYRORegister(BMI088 *imu, uint16_t reg,
									   uint8_t *data) {
	return HAL_I2C_MEM_READ (imu->i2cHandle, BMI088_I2C_GYRO_ADDR, reg, I2C_MEMADD_SIZE_8BIT,data, 1, HAL_MAX_DELAY);
}


HAL_StatusTypeDef BMI088_readGYRORegisters(BMI088 *imu, uint16_t reg,
										uint8_t *data, uint8_t length) {
	return HAL_I2C_MEM_READ (imu->i2cHandle, BMI088_I2C_GYRO_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);

}

HAL_StatusTypeDef BMI088_writeGYRORegister(BMI088 *imu, uint16_t reg,
										uint8_t *data) {
	return HAL_I2C_MEM_WRITE (imu->i2cHandle, BMI088_I2C_GYRO_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BMI088_transmitCommandGyro(BMI088 *imu,uint8_t *data) {
	return HAL_I2C_Master_Transmit(imu -> i2cHandle, BMI088_I2C_GYRO_ADDR, data,
									   1 ,HAL_MAX_DELAY);
}


