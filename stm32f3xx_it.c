/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file stm32f3xx_it.c
* @brief Interrupt Service Routines.
******************************************************************************
* @attention
*
* Copyright (c) 2023 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
******************************************************************************
*/

#include "main.h"
#include "stm32f3xx_it.h"
#include "bmi160_driver.h"
#include "bmp_driver.h"
#include "utility.h"
#include "stdio.h"
#include "main.h"

extern BMI160 imu1;
extern BMI160 imu2;

extern BMP3 barometer1;
extern BMP3_CALIB_DATA calib_baro1;

extern BMP3 barometer2;
extern BMP3_CALIB_DATA calib_baro2;

extern EEPROM eeprom;
extern STATE_MACHINE states;

extern TIM_HandleTypeDef htim1;

/******************************************************************************/
/* Cortex-M4 Processor Interruption and Exception Handlers */
/******************************************************************************/
/**
* @brief This function handles Non maskable interrupt.
*/
 // @brief This function handles Non maskable interrupt.
 void NMI_Handler(void) {
 while (1) {
	 }
 }
// @brief This function handles Hard fault interrupt.

void HardFault_Handler(void) {
 while (1) {
	 }
 }

// @brief This function handles Memory management fault.

void MemManage_Handler(void) {
 while (1) {
	 }
 }

// @brief This function handles Pre-fetch fault, memory access fault.

void BusFault_Handler(void) {
 while (1){
	 }
 }

// @brief This function handles Undefined instruction or illegal state.

void UsageFault_Handler(void) {
 while (1) {
   }
 }

// @brief This function handles Debug monitor.
 void DebugMon_Handler(void) {
 }

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers */
/* Add here the Interrupt Handlers for the used peripherals. */
/* For the available peripheral interrupt handler names, */
/* please refer to the startup file (startup_stm32f3xx.s). */
/******************************************************************************/

