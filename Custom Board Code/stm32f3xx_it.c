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

/**
 * @brief This function handles EXTI line0 interrupt.
 */
void EXTI0_IRQHandler(void) {
/* USER CODE BEGIN EXTI0_IRQn 0 */
 #if _DEBUG_MODE >= LIGHT_DEBUG
   printf("USER BUTTON PRESSED \n");
 #endif
 if(states.state < FLY) {
  init_ground_data(&barometer1, &eeprom);
  if(barometer2.i2cHandle->Instance == I2C2){
    init_ground_data(&barometer2, &eeprom);
  }
    states.state = READY;
    set_state_eeprom(&states, &eeprom);
} else {
   states.state = INIT;
   states.second_event_primer_state = RESET;
   set_state_eeprom(&states, &eeprom);
   set_second_event_primer_state_eeprom(&states, &eeprom);
}
 HAL_GPIO_EXTI_IRQHandler(USER_BUTTON_Pin);
}

/**
* @brief This function handles EXTI line2 and Touch Sense controller interrupts.
*/
 void EXTI2_TSC_IRQHandler(void) {
   #if _DEBUG_MODE >= FULL_DEBUG
      printf("baro1 \n");
   #endif

   BMP3_get_pressure(&barometer1, &calib_baro1);
   if(barometer2.i2cHandle->Instance == I2C2) {
      BMP3_get_pressure(&barometer2, &calib_baro2);
      #if _DEBUG_MODE >= FULL_DEBUG
         printf("baro2 \n");
      #endif
   }
   HAL_GPIO_EXTI_IRQHandler(I2C1_INT1_Pin);
}

 /**
 * @brief This function handles EXTI line4 interrupt.
 */
 void EXTI4_IRQHandler(void) {
	 #if _DEBUG_MODE >= LIGHT_DEBUG
		 printf("HIGH G INTERRUPT I2C1 !\n");
	 #endif

	 #if _DEMO_MODE
	 //for the presenation demo
		 if(states.state == FLY) {
			 IWDG->KR = 0xAAAA;
			 set_second_event_current_impulse();
			 printf("second event demo");
			 HAL_Delay(DEMO_DURATION);
			 reset_second_event_current_impulse();
		 }
	 #endif

 	if(states.state == READY) {
		 states.state = FLY;
		 set_state_eeprom(&states, &eeprom);
	 }

	 HAL_GPIO_EXTI_IRQHandler(I2C1_INT3_Pin);
 }

 /**
 * @brief This function handles EXTI line[9:5] interrupts.
 */
void EXTI9_5_IRQHandler(void) {
	#if _DEBUG_MODE >= FULL_DEBUG
		printf("imu1 \n");
	#endif
	BMI160_get_accel(&imu1);
	
	if(imu2.i2cHandle -> Instance == I2C2) {
		BMI160_get_accel(&imu2);
		#if _DEBUG_MODE >= FULL_DEBUG
		printf("imu2 \n");
		#endif
	}
	HAL_GPIO_EXTI_IRQHandler(I2C1_INT2_Pin);
}

 /**
 * @brief This function handles TIM1 update and TIM16 interrupts.
 */
void TIM1_UP_TIM16_IRQHandler(void) {
 	HAL_TIM_IRQHandler(&htim1);
 }

 /**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler(void) {
	#if _DEBUG_MODE >= LIGHT_DEBUG
		printf("HIGH G INTTERUPT I2C2 !\n");
	#endif
	
	#if _DEMO_MODE
	//for the presenation demo
		if(states.state == FLY) {
			IWDG->KR = 0xAAAA;
			set_second_event_current_impulse();
			printf("second event demo");
			HAL_Delay(DEMO_DURATION);
			reset_second_event_current_impulse();
		}
	#endif
	if(states.state == READY) {
		states.state = FLY;
		set_state_eeprom(&states, &eeprom);
	}
	HAL_GPIO_EXTI_IRQHandler(I2C2_INT3_Pin);
}
