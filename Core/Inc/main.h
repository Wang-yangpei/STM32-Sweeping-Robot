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
#define PWMB_Pin GPIO_PIN_0
#define PWMB_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_1
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_2
#define AIN2_GPIO_Port GPIOA
#define PWMA_Pin GPIO_PIN_3
#define PWMA_GPIO_Port GPIOA
#define BUMPER_LEFT_Pin GPIO_PIN_4
#define BUMPER_LEFT_GPIO_Port GPIOA
#define BUMPER_LEFT_EXTI_IRQn EXTI4_IRQn
#define BUMPER_FRONT_Pin GPIO_PIN_5
#define BUMPER_FRONT_GPIO_Port GPIOA
#define BUMPER_FRONT_EXTI_IRQn EXTI9_5_IRQn
#define BUMPER_RIGHT_Pin GPIO_PIN_6
#define BUMPER_RIGHT_GPIO_Port GPIOA
#define BUMPER_RIGHT_EXTI_IRQn EXTI9_5_IRQn
#define CLIFF_LEFT_Pin GPIO_PIN_7
#define CLIFF_LEFT_GPIO_Port GPIOA
#define CLIFF_LEFT_EXTI_IRQn EXTI9_5_IRQn
#define BIN1_Pin GPIO_PIN_0
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_1
#define BIN2_GPIO_Port GPIOB
#define ECHO_Pin GPIO_PIN_10
#define ECHO_GPIO_Port GPIOB
#define TRIG_Pin GPIO_PIN_11
#define TRIG_GPIO_Port GPIOB
#define WALL_RIGHT_Pin GPIO_PIN_13
#define WALL_RIGHT_GPIO_Port GPIOB
#define CLIFF_RIGHT_Pin GPIO_PIN_11
#define CLIFF_RIGHT_GPIO_Port GPIOA
#define CLIFF_RIGHT_EXTI_IRQn EXTI15_10_IRQn
#define CLIFF_FRONT_Pin GPIO_PIN_12
#define CLIFF_FRONT_GPIO_Port GPIOA
#define CLIFF_FRONT_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
