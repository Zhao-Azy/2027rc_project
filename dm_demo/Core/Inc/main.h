/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define IO_0_Pin GPIO_PIN_0
#define IO_0_GPIO_Port GPIOA
#define IO_1_Pin GPIO_PIN_1
#define IO_1_GPIO_Port GPIOA
#define IO_2_Pin GPIO_PIN_2
#define IO_2_GPIO_Port GPIOA
#define IO_3_Pin GPIO_PIN_3
#define IO_3_GPIO_Port GPIOA
#define IO_4_Pin GPIO_PIN_4
#define IO_4_GPIO_Port GPIOC
#define IO_5_Pin GPIO_PIN_5
#define IO_5_GPIO_Port GPIOC
#define IO_6_Pin GPIO_PIN_0
#define IO_6_GPIO_Port GPIOB
#define IO_7_Pin GPIO_PIN_1
#define IO_7_GPIO_Port GPIOB
#define IO_8_Pin GPIO_PIN_2
#define IO_8_GPIO_Port GPIOB
#define IO_9_Pin GPIO_PIN_7
#define IO_9_GPIO_Port GPIOE
#define IO_10_Pin GPIO_PIN_8
#define IO_10_GPIO_Port GPIOE
#define IO_11_Pin GPIO_PIN_9
#define IO_11_GPIO_Port GPIOE
#define IO_16_Pin GPIO_PIN_10
#define IO_16_GPIO_Port GPIOE
#define IO_18_Pin GPIO_PIN_12
#define IO_18_GPIO_Port GPIOE
#define IO_19_Pin GPIO_PIN_13
#define IO_19_GPIO_Port GPIOE
#define IO_20_Pin GPIO_PIN_14
#define IO_20_GPIO_Port GPIOE
#define IO_21_Pin GPIO_PIN_15
#define IO_21_GPIO_Port GPIOE
#define IO_22_Pin GPIO_PIN_10
#define IO_22_GPIO_Port GPIOB
#define IO_23_Pin GPIO_PIN_11
#define IO_23_GPIO_Port GPIOB
#define IO_15_Pin GPIO_PIN_11
#define IO_15_GPIO_Port GPIOC
#define IO_14_Pin GPIO_PIN_12
#define IO_14_GPIO_Port GPIOC
#define IO_12_Pin GPIO_PIN_3
#define IO_12_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
