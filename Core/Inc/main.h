/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  *************************************************************************************************************
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define speed_sensor_Pin GPIO_PIN_13
#define speed_sensor_GPIO_Port GPIOC
#define speed_sensor_EXTI_IRQn EXTI15_10_IRQn
#define LCD_D0_Pin GPIO_PIN_0
#define LCD_D0_GPIO_Port GPIOA
#define LCD_D1_Pin GPIO_PIN_1
#define LCD_D1_GPIO_Port GPIOA
#define LCD_D2_Pin GPIO_PIN_2
#define LCD_D2_GPIO_Port GPIOA
#define LCD_D3_Pin GPIO_PIN_3
#define LCD_D3_GPIO_Port GPIOA
#define LCD_D4_Pin GPIO_PIN_4
#define LCD_D4_GPIO_Port GPIOA
#define LCD_D5_Pin GPIO_PIN_5
#define LCD_D5_GPIO_Port GPIOA
#define LCD_D6_Pin GPIO_PIN_6
#define LCD_D6_GPIO_Port GPIOA
#define LCD_D7_Pin GPIO_PIN_7
#define LCD_D7_GPIO_Port GPIOA
#define lcd_adr_Pin GPIO_PIN_0
#define lcd_adr_GPIO_Port GPIOB
#define LCD_RD_Pin GPIO_PIN_1
#define LCD_RD_GPIO_Port GPIOB
#define lcd_reset_Pin GPIO_PIN_2
#define lcd_reset_GPIO_Port GPIOB
#define lcd_chip_sel_Pin GPIO_PIN_10
#define lcd_chip_sel_GPIO_Port GPIOB
#define check_led_Pin GPIO_PIN_11
#define check_led_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_8
#define Buzzer_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

#define ODO_ADDRESS_EEPROM 0X20
#define TRIP_ADDRESS_EEPROM 0X30
#define LAST_STATE_ADDRESS_EEPROM 0X40
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
