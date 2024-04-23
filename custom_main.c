main.c Monday, 5 June 2023, 10:05
/**
******************************************************************************
* @file : main.c
* @brief : Main program body
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
*
* Semester project made by Maxime Leriche for the EPFL Rocket Team and
* supervised by Theo Kluter of the processor architecture lab at EPFL.
* This code control a custom embedded system triggering the second reefing
* event directly on the rocket parachute.
*/

#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "stdio.h"
#include "stm32f3xx.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_hal_i2c.h"
#include "bmp_driver.h"
#include "bmi160_driver.h"
#include "eeprom_driver.h"
#include "utility.h"
#include <stdint.h>
#include <string.h>
#include <stdarg.h>

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

osThreadId defaultTaskHandle;
osThreadId main_threadHandle;
osThreadId data_log_threadHandle;
osThreadId user_interface_Handle;

// typedef structure
STATE_MACHINE states;

BMP3 barometer1;
BMP3_CALIB_DATA calib_baro1;

BMP3 barometer2;
BMP3_CALIB_DATA calib_baro2;

BMI160 imu1;
BMI160 imu2;

EEPROM eeprom;

//variables for FatFs
FATFS FatFs; //Fatfs handle
FIL fil; //File handle
FRESULT fres; //Result after operations

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void const * argument);
void Start_main_thread(void const * argument);
void Start_data_log_thread(void const * argument);
void Start_user_interface_thread(void const * argument);

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

 PUTCHAR_PROTOTYPE
/*https://forum.digikey.com/t/easily-use-printf-on-stm32/20157
 * to redirect the printf to the huart port for the debugger*/
 {
 HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
 return ch;
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

/* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_FATFS_Init();
  MX_IWDG_Init();

  HAL_Delay(2000); //a short delay is important to let the SD card settle
  IWDG->KR = 0xAAAA; //reload watchdog

  /*for interrupt setup reason line 1 as to be set second and line 2 can not be
  * used without line 1, sensor data is retrieved thanks to hardware interrupt
  * trigger by sensor itself*/

  EEPROM_init(&eeprom, EEPROM_I2C_ADDR_1, &hi2c2);
  BMI160_init(&imu2, BMI160_I2C_ADDR_PRIM, &hi2c2, 4, 2000); //line i2c2
  BMI160_init(&imu1, BMI160_I2C_ADDR_PRIM, &hi2c1, 4, 2000); //line i2c1
  
  BMP3_initialisation(&barometer2,&eeprom, BMP3_ADDR_I2C_PRIM ,&hi2c2 ,
  &calib_baro2); //line i2c2
  
  BMP3_initialisation(&barometer1,&eeprom, BMP3_ADDR_I2C_PRIM ,&hi2c1 ,
   &calib_baro1); //line i2c1

  /* Safety measure to retrieve the primary flight data from eeprom in case of
  * unexpected reboot during flight, the ground altitude is retrieve in
  * BMP3_initialisation */
  
  get_state_eeprom(&states, &eeprom);
  get_second_event_primer_state_eeprom(&states, &eeprom);
  
  /* Thanks to kiwih for the library and example available at
  * https://github.com/kiwih/cubeide-sd-cardfor the sd card implementation*/

  //Open the file system
  DSTATUS fres1;
  fres1 = disk_status (0);
  printf("status %u", fres1);
  fres = f_mount(&FatFs, "", 1); //1 = mount now
  if (fres != FR_OK){
    printf("f_mount error (%i)\r\n", fres); //was myprintf
  } 
  fres = f_close(&fil);

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
  
  /* definition and creation of main_thread */
  osThreadDef(main_thread, Start_main_thread, osPriorityNormal, 0, 1024);
  main_threadHandle = osThreadCreate(osThread(main_thread), NULL);
  
  /* definition and creation of data_log_thread */
  osThreadDef(data_log_thread, Start_data_log_thread, osPriorityLow, 0, 1024);
  data_log_threadHandle = osThreadCreate(osThread(data_log_thread), NULL);
  
  /* definition and creation of user_interface_ */
  osThreadDef(user_interface_, Start_user_interface_thread, osPriorityLow, 0, 128);
  user_interface_Handle = osThreadCreate(osThread(user_interface_), NULL);
  
  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */

  while (1){
    }
}
/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
  |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}
 
/**
* @brief I2C1 Initialization Function
* @param None
* @retval None
*/
static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
/** Configure Analogue filter
*/
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

/** Configure Digital filter
*/
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    Error_Handler();
  }
}
/**
* @brief I2C2 Initialization Function
* @param None
* @retval None
*/
static void MX_I2C2_Init(void) {
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }
/** Configure Analogue filter
*/
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }
/** Configure Digital filter
*/
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
    Error_Handler();
  }
}
/**
* @brief IWDG Initialization Function
* @param None
* @retval None
*/
static void MX_IWDG_Init(void) {
  //stop watchdog while debugging
   DBGMCU->APB1FZ = DBGMCU->APB1FZ | DBGMCU_APB1_FZ_DBG_IWDG_STOP;
  
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
    Error_Handler();
  }
}


/**
* @brief SPI1 Initialization Function
* @param None
* @retval None
*/
static void MX_SPI1_Init(void) {
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
   hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
}
/**
* @brief USART1 Initialization Function
* @param None
* @retval None
*/
static void MX_USART1_UART_Init(void) {
huart1.Instance = USART1;
huart1.Init.BaudRate = 115200;
huart1.Init.WordLength = UART_WORDLENGTH_8B;
huart1.Init.StopBits = UART_STOPBITS_1;
huart1.Init.Parity = UART_PARITY_NONE;
huart1.Init.Mode = UART_MODE_TX_RX;
huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
huart1.Init.OverSampling = UART_OVERSAMPLING_16;
huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
if (HAL_UART_Init(&huart1) != HAL_OK)
 {
Error_Handler();
}
}
/**
* @brief USB Initialization Function
* @param None
* @retval None
*/
static void MX_USB_PCD_Init(void) {
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) {
    Error_Handler();
  }
}
/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SECOND_EVENT_Pin|BUZZER_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  
  /*Configure GPIO pins : I2C2_INT1_Pin I2C2_INT2_Pin */
  GPIO_InitStruct.Pin = I2C2_INT1_Pin|I2C2_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /*Configure GPIO pin : I2C2_INT3_Pin */
  GPIO_InitStruct.Pin = I2C2_INT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(I2C2_INT3_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : SD_CS_Pin LED1_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*Configure GPIO pins : I2C1_INT1_Pin I2C1_INT3_Pin I2C1_INT2_Pin */
  GPIO_InitStruct.Pin = I2C1_INT1_Pin|I2C1_INT3_Pin|I2C1_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*Configure GPIO pins : SECOND_EVENT_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = SECOND_EVENT_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*Configure GPIO pin : LED_2_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_2_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pin : SYS_SWO_Pin */
  GPIO_InitStruct.Pin = SYS_SWO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SYS_SWO_GPIO_Port, &GPIO_InitStruct);
  
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument) {
  /* Infinite loop */
  for(;;) {
    osDelay(1000);
  }
}
/**
* @brief Function implementing the main_thread thread.
* @param argument: Not used
* @retval None
*/
void Start_main_thread(void const * argument) {
  //one per barometer for redundancy
  FLIGHT_DATA flight_data1[FLIGHT_DATA_BUFFER_SIZE];
  FLIGHT_DATA flight_data2[FLIGHT_DATA_BUFFER_SIZE];
  uint8_t counter1 = 0;
  uint8_t counter2 = 0;
  
  for(;;) {
    temporal_flight_data_compiler(&barometer1, flight_data1, &counter1);
    second_event_height_primer_state(&states, flight_data1,
    &eeprom, &counter1);
  
    reefing_second_event(flight_data1, &states, &counter1);
  
    temporal_flight_data_compiler(&barometer2, flight_data2, &counter2);
    second_event_height_primer_state(&states, flight_data2, &eeprom, &counter2);
  
    reefing_second_event(flight_data2, &states, &counter2);
  
    #if _DEBUG_MODE >= LIGHT_DEBUG
    printf("time %lu \n", flight_data1[counter1].time);
    printf("altitude 1 %ld \n", flight_data1[counter1].height);
    printf("altitude 2 %ld \n", flight_data2[counter2].height);
    #endif
  
    osDelay(1000);
  }
}

/**
* @brief Function implementing the data_log_thread thread.
* @param argument: Not used
* @retval None
*/
void Start_data_log_thread(void const * argument) {
  char baro_sd_buffer1[BMP3_SD_BUFFER_SIZE];
  uint8_t baro_buffer_index1 = 0;
  
  char baro_sd_buffer2[BMP3_SD_BUFFER_SIZE];
  uint8_t baro_buffer_index2 = 0;
  
  char imu_sd_buffer1[BMI160_SD_BUFFER_SIZE];
  uint8_t imu_buffer_index1 = 0;
  
  char imu_sd_buffer2[BMI160_SD_BUFFER_SIZE];
  uint8_t imu_buffer_index2 = 0;

  for(;;) {
    if(states.state == FLY) {
      BMI160_datalog(&imu1, &fil, fres, imu_sd_buffer1,
      &imu_buffer_index1, "imu1.txt");
      
      BMI160_datalog(&imu2, &fil, fres, imu_sd_buffer2,
      &imu_buffer_index2, "imu2.txt");
      
      BMP3_datalog(&barometer1, &fil, fres, baro_sd_buffer1,
      &baro_buffer_index1, "baro1.txt");
      
      BMP3_datalog(&barometer2, &fil, fres, baro_sd_buffer2,
      &baro_buffer_index2, "baro2.txt");
    }

  #if _DEBUG_MODE >= LIGHT_DEBUG
  printf("state %u \n", states.state);
  #endif
  
  osDelay(1000);
  }
}

/**
* @brief Function implementing the user_interface_ thread.
* @param argument: Not used
* @retval None
*/
void Start_user_interface_thread(void const * argument) {
  for(;;) {
    if(states.state == INIT) {
      GREEN_LED_on();
      RED_LED_toggle();
      osDelay(500);
    } else if(states.state == READY) {
      GREEN_LED_toggle();
      RED_LED_off();
      osDelay(200);
    } else {
      GREEN_LED_toggle();
      RED_LED_off();
      osDelay(1000);
  }
    IWDG->KR = 0xAAAA;
  }
}

/**
* @brief Period elapsed callback in non blocking mode
* @note This function is called when TIM1 interrupt took place, inside
* HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
* a global variable "uwTick" used as application time base.
* @param htim : TIM handle
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
}

/**
* @brief This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void) {
   __disable_irq();
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
*/
 void assert_failed(uint8_t *file, uint32_t line)
 {
 /* User can add his own implementation to report the file name and line number,
 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
 }
 #endif /* USE_FULL_ASSERT */
