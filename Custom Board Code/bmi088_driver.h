/*
 * bmi088_driver.h
 *
 *  Created on: May 1, 2024
 *      Author: psanc
 */

#ifndef INC_BMI088_DRIVER_H_
#define INC_BMI088_DRIVER_H_
/*
 * INCLUDES
 */
#include "stm32f3xx_hal_i2c.h"
#include "stm32f3xx.h"

/*
 * DEFINES
 */
#define BMI088_I2C_ACCL_ADDR (0x18 << 1) //0x18 SD01 pulled to GND - 0x19 SD01 pulled to VDDIO Page 48
#define BMI088_I2C_GYRO_ADDR (0x68 << 1) //0x68 SD02 pulled to GND - 0x69 SD02 pulled to VDDIO Page 15

#define BMI088_ACCL_CHIP_ID 0x1E
#define BMI088_ACCL_GRYO_ID 0x0F

/*
 * #define BMI088_MEMS_ID
 * #define BMI088_PART_ID
 * These don't seem to exist for the BMI088
 */


/*
 * REGISTERS (Page 25 for Accelerometer, Page 36 for Gyroscope)
 */

//Accelerometer
#define BMI088_ACC_SOFTRESET 0x7E
#define BMI088_ACC_PWR_CNTRL 0x7D
#define BMI088_ACC_PWR_CONF 0x7C
#define BMI088_ACC_SELF_TEST 0x6D

#define BMI088_ACC_INT_MAP_DATA 0x58
#define BMI088_ACC_INT2_IO_CTRL 0x54
#define BMI088_ACC_INT1_IO_CTRL 0x53
#define BMI088_ACC_INT_STAT_1 0x1D

#define BMI088_ACC_RANGE 0x41
#define BMI088_ACC_CONF 0x40


#define BMI088_ACC_SENSORTIME_2 0x1A
#define BMI088_ACC_SENSORTIME_1 0x19
#define BMI088_ACC_SENSORTIME_0 0x18

#define BMI088_ACC_Z_MSB 0x17
#define BMI088_ACC_Z_LSB 0x16
#define BMI088_ACC_Y_MSB 0x15
#define BMI088_ACC_Y_LSB 0x14
#define BMI088_ACC_X_MSB 0x13
#define BMI088_ACC_X_LSB 0x12

#define BMI088_ACC_STATUS 0x03
#define BMI088_ACC_ERR_REG 0x02
#define ACC_CHIP_ID 0x00

#define BMI088_ACC_TEMP_LSB 0x23
#define BMI088_ACC_TEMP_MSB 0x22

#define BMI088_ACC_FIFO_CONFIG_1 0x49
#define BMI088_ACC_FIFO_CONFIG_0 0x48
#define BMI088_ACC_FIFO_WTM_1 0x47
#define BMI088_ACC_FIFO_WTM_0 0x46
#define BMI088_ACC_FIFO_DATA 0x26
#define BMI088_ACC_FIFO_LENGTH_1 0x25
#define BMI088_ACC_FIFO_LENGTH_0 0x24

#define BMI088_ACC_DOWNS 0x45


//Gyroscope
#define BMI088_GYRO_SOFT_RESET 0x14
#define BMI088_GYRO_SELF_TEST 0x3C
#define BMI088_GYRO_INT_STAT_1 0x0A

#define BMI088_GYRO_INT_CTRL 0x15
#define BMI088_GYR_INT3_INT4_IO_MAP 0x18
#define BMI088_GYR_INT3_INT4_IO_CONF 0x16


#define BMI088_GYRO_LPM1 0x11
#define BMI088_GYRO_BANDWIDTH 0x10
#define BMI088_GYRO_RANGE 0x0F


#define BMI088_RATE_Z_MSB 0x07
#define BMI088_RATE_Z_LSB 0x06
#define BMI088_RATE_Y_MSB 0x05
#define BMI088_RATE_Y_LSB 0x04
#define BMI088_RATE_X_MSB 0x03
#define BMI088_RATE_X_LSB 0x02

#define BMI088_GYRO_CHIP_ID 0x00

#define BMI088_GYR_FIFO_DATA 0x3F
#define BMI088_GYR_FIFO_CONFIG_1 0x3E
#define BMI088_GYR_FIFO_CONFIG_0 0x3D
#define BMI088_GYR_FIFO_EXT_INT_S 0x34
#define BMI088_GYR_FIFO_WM_EN 0x1E
#define BMI088_GYR_FIFO_STATUS 0x0E


// BMI088 structure
typedef struct {
	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;
	uint8_t i2c_address;

	GPIO_TypeDef *interrupt_port_1;
	uint16_t interrupt_pin_1;
	GPIO_TypeDef *interrupt_port_2;
	uint16_t interrupt_pin_2;

	uint8_t accel_range; //2,4,8,16 g
	uint16_t gyro_range;
	int16_t raw_gyro_x;
	int16_t raw_gyro_y;
	int16_t raw_gyro_z;
	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;

	float temperature_imu;
}BMI088;

HAL_StatusTypeDef BMI088_writeRegister(BMI088 *imu, uint16_t reg,
										uint8_t *data);

HAL_StatusTypeDef BMI088_transmitCommand(BMI088 *imu,uint8_t *data);

HAL_StatusTypeDef BMI088_readRegister(BMI088 *imu, uint16_t reg,
									   uint8_t *data);

HAL_StatusTypeDef BMI088_readRegisters(BMI088 *imu, uint16_t reg,
										uint8_t *data, uint8_t length);

void BMI088_Initialise(BMI088 *imu, uint8_t i2c_address, I2C_HandleTypeDef *i2cHandle,
				 uint8_t accel_range, uint16_t gyro_range);

void BMI088_get_temp(BMI088 *imu);

void BMI088_compensate_temp(BMI088 *imu, uint16_t raw_temperature);

void BMI088_get_accel(BMI088 *imu);

void BMI088_get_gyro(BMI088 *imu);

void BMI088_interrupt_port_setup(BMI088 *imu);

void BMI088_interrupt_setup(BMI088 *imu);

void BMI088_setup_normal_mode(BMI088 *imu);

void BMI088_accel_conversion(BMI088 *imu, int16_t raw_acc_x, int16_t raw_acc_y,
							 int16_t raw_acc_z);

void BMI088_GYRO_conversion(BMI088 *imu, int16_t raw_acc_x, int16_t raw_acc_y,
							int16_t raw_acc_z);

void BMI088_datalog(BMI088 *imu, FIL *fil, FRESULT fres, char *sd_buffer,
				    uint8_t* buffer_index, char file_name[]);

#endif /* INC_BMI088_DRIVER_H_ */
