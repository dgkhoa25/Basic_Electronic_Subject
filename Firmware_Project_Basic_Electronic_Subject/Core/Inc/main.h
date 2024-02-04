/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define PWM_IN1_Pin GPIO_PIN_0
#define PWM_IN1_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_1
#define IN2_GPIO_Port GPIOA
#define CLK1_Pin GPIO_PIN_0
#define CLK1_GPIO_Port GPIOB
#define DT1_Pin GPIO_PIN_1
#define DT1_GPIO_Port GPIOB
#define MAC200_Pin GPIO_PIN_12
#define MAC200_GPIO_Port GPIOB
#define START_Pin GPIO_PIN_13
#define START_GPIO_Port GPIOB
#define STOP_Pin GPIO_PIN_14
#define STOP_GPIO_Port GPIOB
#define MAC75_Pin GPIO_PIN_15
#define MAC75_GPIO_Port GPIOB
#define Servo__1_Pin GPIO_PIN_8
#define Servo__1_GPIO_Port GPIOA
#define Servo_2_Pin GPIO_PIN_9
#define Servo_2_GPIO_Port GPIOA
#define Servo_3_Pin GPIO_PIN_10
#define Servo_3_GPIO_Port GPIOA
#define Servo_4_Pin GPIO_PIN_11
#define Servo_4_GPIO_Port GPIOA
#define CLK2_Pin GPIO_PIN_3
#define CLK2_GPIO_Port GPIOB
#define DT2_Pin GPIO_PIN_4
#define DT2_GPIO_Port GPIOB
#define CLK3_Pin GPIO_PIN_5
#define CLK3_GPIO_Port GPIOB
#define DT3_Pin GPIO_PIN_6
#define DT3_GPIO_Port GPIOB
#define CLK4_Pin GPIO_PIN_7
#define CLK4_GPIO_Port GPIOB
#define DT4_Pin GPIO_PIN_8
#define DT4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
