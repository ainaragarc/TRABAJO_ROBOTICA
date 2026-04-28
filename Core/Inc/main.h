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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENC_M1_V_Pin GPIO_PIN_0
#define ENC_M1_V_GPIO_Port GPIOA
#define ENC_M2_B_Pin GPIO_PIN_1
#define ENC_M2_B_GPIO_Port GPIOA
#define ENC_BASE_V_Pin GPIO_PIN_6
#define ENC_BASE_V_GPIO_Port GPIOA
#define ENC_BASE_B_Pin GPIO_PIN_7
#define ENC_BASE_B_GPIO_Port GPIOA
#define m1_Pin GPIO_PIN_9
#define m1_GPIO_Port GPIOE
#define m2_Pin GPIO_PIN_11
#define m2_GPIO_Port GPIOE
#define m3_Pin GPIO_PIN_13
#define m3_GPIO_Port GPIOE
#define EN_Pin GPIO_PIN_10
#define EN_GPIO_Port GPIOB
#define m4_Pin GPIO_PIN_6
#define m4_GPIO_Port GPIOC
#define FC_Izquierda_Pin GPIO_PIN_8
#define FC_Izquierda_GPIO_Port GPIOA
#define FC_Derecha_Pin GPIO_PIN_9
#define FC_Derecha_GPIO_Port GPIOA
#define STEP_Pin GPIO_PIN_6
#define STEP_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_7
#define DIR_GPIO_Port GPIOB
#define ENABLE_Pin GPIO_PIN_8
#define ENABLE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
