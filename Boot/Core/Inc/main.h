/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7rsxx_hal.h"
#include "stm32h7rsxx_nucleo.h"
#include <stdio.h>

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
#define CLK_Pin GPIO_PIN_3
#define CLK_GPIO_Port GPIOF
#define LAT_Pin GPIO_PIN_4
#define LAT_GPIO_Port GPIOF
#define B2_Pin GPIO_PIN_11
#define B2_GPIO_Port GPIOE
#define R1_Pin GPIO_PIN_6
#define R1_GPIO_Port GPIOE
#define OE_Pin GPIO_PIN_5
#define OE_GPIO_Port GPIOF
#define B1_Pin GPIO_PIN_7
#define B1_GPIO_Port GPIOE
#define C_Pin GPIO_PIN_2
#define C_GPIO_Port GPIOP
#define D_Pin GPIO_PIN_3
#define D_GPIO_Port GPIOP
#define A_Pin GPIO_PIN_0
#define A_GPIO_Port GPIOP
#define R2_Pin GPIO_PIN_9
#define R2_GPIO_Port GPIOE
#define B_Pin GPIO_PIN_1
#define B_GPIO_Port GPIOP
#define G1_Pin GPIO_PIN_8
#define G1_GPIO_Port GPIOE
#define G2_Pin GPIO_PIN_10
#define G2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
