/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
void ADC_Calibrate(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define D7_PIN_Pin GPIO_PIN_6
#define D7_PIN_GPIO_Port GPIOA
#define D6_PIN_Pin GPIO_PIN_7
#define D6_PIN_GPIO_Port GPIOA
#define D5_PIN_Pin GPIO_PIN_0
#define D5_PIN_GPIO_Port GPIOB
#define D4_PIN_Pin GPIO_PIN_7
#define D4_PIN_GPIO_Port GPIOC
#define RW_PIN_Pin GPIO_PIN_3
#define RW_PIN_GPIO_Port GPIOB
#define EN_PIN_Pin GPIO_PIN_4
#define EN_PIN_GPIO_Port GPIOB
#define RS_PIN_Pin GPIO_PIN_5
#define RS_PIN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
