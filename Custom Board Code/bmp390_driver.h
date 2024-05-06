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
#define BMP390_CALIB_DATA 0x31 // Technically from 0x31 to 0x57
#define BMP390_CMD 0x7E

//Error Status Macros
#define BMP390_FATAL_ERROR 0x01
#define BMP390_COMMAND_ERROR 0x02
#define BMP390_CONF_ERROR 0x04

//Status Macros
#define BMP390_STATUS_CMD_RDY 0x01 //Command decoder status - 0 = command in progress, 1 = ready for new command
#define BMP390_STATUS_DRDY_PRESS 0x02 //Data ready for pressure, reset when one pressure DATA register is read
#define BMP390_STATUS_DRDY_TEMP 0x04 //Data ready for temperature, reset when one temperature DATA register is read

//Sensor Status Flags
#define BMP390_EVENT_POR_DETECTED 0x01 //1 after device power up or soft reset. Clear on read.
#define BMP390_EVENT_ITF_ACT_PT 0x02 // 1 when a serial interface transaction occurs during a pressure or temperature conversion. Clear on read.

// Interrupt Status Flags
#define BMP390_INT_STATUS_FWM_INT 0x01 //FIFO Watermark Interrupt
#define BMP390_INT_STATUS_FFUL_INT 0x02 // Full FIFO Interrupt
#define BMP390_INT_STATUS_DRDRY 0x08  //Data Ready Interrupt

// Interrupt Control
#define BMP390_INT_OD 0x01 // Configure Output | Push-Pull = 0, Open Drain = 1
#define BMP390_INT_LEVEL 0x02 // Level of Int Pin | Active Low = 0, Active High = 1
#define BMP390_INT_LATCH 0x04 // Latching of interrupts for int pin and INT_STATUS register | Disabled = 0, Enabled = 1
#define BMP390_INT_FWTM_EN 0x08 // enable FIFO watermark reached interrupt for int pin and INT_STATUS register | Disabled = 0, Enabled = 1
#define BMP390_INT_FFUL_EN 0x10 //enable FIFO full interrupt for int pin and INT_STATUS register | Disabled = 0, Enabled = 1
#define BMP390_INT_DS 0x20 // 0 = Low, 1 = High
#define BMP390_INT_DRDY_EN 0x40 // Enable Temperature/Pressure Data ready interrupt for INT pin and INT_STATUS | Disabled = 0, Enabled = 1

// Serial Interface Settings (SPI and Watchdog Timer)
#define BMP390_SPI3 0x01 // 0 = SPI 4 Wire mode, 1 = SPI 3 Wire mode
#define BMP390_I2C_WDT_EN 0x02 // Enable the watchdog timer | 0 = Disabled, 1 = Enabled
#define BMP390_I2C_WDT_SEL 0x04 // Timer period for I2C Watchdog | 0 = 1.25ms, 1 = 40ms

//Power Control - Disabled and Enabling Measurements, and can set measurement mode
#define BMP390_PWR_PRESS_EN 0x01 // 0 = Disable Pressure Sensor, 1 = Enable Pressure Sensor
#define BMP390_PWR_TEMP_EN 0x02 // 0 = Disable Temperature Sensor, 1 = Enable Temperature Sensor

// Power mode macros
#define BMP3_MODE_SLEEP 0x00
#define BMP3_MODE_FORCED 0x01
#define BMP3_MODE_NORMAL 0x03

// OSR Macros (Oversampling) - These settings apply for both pressure and temperature
#define BMP390_NO_OVERSAMPLING 0x00
#define BMP390_OVERSAMPLING_2X 0x01
#define BMP390_OVERSAMPLING_4X 0x02
#define BMP390_OVERSAMPLING_8X 0x03
#define BMP390_OVERSAMPLING_16X 0x04
#define BMP390_OVERSAMPLING_32X 0x05

//Output Data Rate Macros
#define BMP390_ODR_200_HZ 0x00
#define BMP3_ODR_100_HZ 0x01
#define BMP3_ODR_50_HZ 0x02
#define BMP3_ODR_25_HZ 0x03
#define BMP3_ODR_12_5_HZ 0x04
#define BMP3_ODR_6_25_HZ 0x05
#define BMP3_ODR_3_1_HZ 0x06
#define BMP3_ODR_1_5_HZ 0x07
#define BMP3_ODR_0_78_HZ 0x08
#define BMP3_ODR_0_39_HZ 0x09
#define BMP3_ODR_0_2_HZ 0x0A
#define BMP3_ODR_0_1_HZ 0x0B
#define BMP3_ODR_0_05_HZ 0x0C
#define BMP3_ODR_0_02_HZ 0x0D
#define BMP3_ODR_0_01_HZ 0x0E
#define BMP3_ODR_0_006_HZ 0x0F
#define BMP3_ODR_0_003_HZ 0x10
#define BMP3_ODR_0_001_HZ 0x11

// IIR Filter Macros (CONFIG)
#define BMP390_IIR_FILTER_DISABLE 0x01
#define BMP390_IIR_FILTER_COEFF_1 0x01
#define BMP390_IIR_FILTER_COEFF_3 0x02
#define BMP390_IIR_FILTER_COEFF_7 0x03
#define BMP390_IIR_FILTER_COEFF_15 0x04
#define BMP390_IIR_FILTER_COEFF_31 0x05
#define BMP390_IIR_FILTER_COEFF_63 0x06
#define BMP390_IIR_FILTER_COEFF_127 0x07

//Soft Reset: Triggers a reset, all user configuration settings are overwritten with their default state
#define BMP390_SOFT_RESET 0xB6

//Clear all FIFO Data Registers - Not FIFO Config Registers
#define BMP390_FIFO_FLUSH 0xB0

//FIFO_CONFIG_1 Macros
#define BMP390_FIFO_MODE 0x01 //Enable = 1/Disable = 0 FIFO
#define BMP390_FIFO_STOP_ON_FULL 0x02 //Stop writing samples into FIFO when FIFO is Full | 0 = Don't stop writing, 1 = Stop Writing
#define BMP390_FIFO_TIME_EN 0x04 // Return sensortime frame after the last valid data frame | 0 = Do not return, 1 = Return
#define BMP390_FIFO_PRESS_EN 0x08 // Store pressure data in FIFO | 0 = Disable, 1 = Enable
#define BMP390_FIFO_TEMP_EN 0x10 // Store temperature data in FIFO | 0 = Disable, 1 = Enable


// FIFO Sub-sampling macros - FIFO_CONFIG_2 Register
#define BMP3_FIFO_NO_SUBSAMPLING 0x00
#define BMP3_FIFO_SUBSAMPLING_2X 0x01
#define BMP3_FIFO_SUBSAMPLING_4X 0x02
#define BMP3_FIFO_SUBSAMPLING_8X 0x03
#define BMP3_FIFO_SUBSAMPLING_16X 0x04
#define BMP3_FIFO_SUBSAMPLING_32X 0x05
#define BMP3_FIFO_SUBSAMPLING_64X 0x06
#define BMP3_FIFO_SUBSAMPLING_128X 0x07

// Page 28 Datasheet - Calibration Data
#define BMP3_PAR_T1_ADR 0x31 //16 bit
#define BMP3_PAR_T2_ADR 0x33 //16 bit
#define BMP3_PAR_T3_ADR 0x35
#define BMP3_PAR_P1_ADR 0x36 //16 bit
#define BMP3_PAR_P2_ADR 0x38 //16 bit
#define BMP3_PAR_P3_ADR 0x3A
#define BMP3_PAR_P4_ADR 0x3B
#define BMP3_PAR_P5_ADR 0x3C //16 bit
#define BMP3_PAR_P6_ADR 0x3E //16 bit
#define BMP3_PAR_P7_ADR 0x40
#define BMP3_PAR_P8_ADR 0x41
#define BMP3_PAR_P9_ADR 0x42 //16 bit
#define BMP3_PAR_P10_ADR 0x44
#define BMP3_PAR_P11_ADR 0x45

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
