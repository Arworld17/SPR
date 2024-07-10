/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM_PSC_APB1 ((APB1_TIMER_CLOCKS/PWM_FREQUENCE)/PWM_RESOLUTION -1)
#define PWM_FREQUENCE 50
#define PWM_RESOLUTION 10000
#define APB1_TIMER_CLOCKS 84000000
#define PWM_DEFAULT_DUTY 5000
#define TIM_PSC_APB2 ((APB2_TIMER_CLOCKS/PWM_FREQUENCE)/PWM_RESOLUTION -1)
#define APB2_TIMER_CLOCKS 168000000
#define LED_RED_Pin GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOF
#define KEY_1_GPIO GPIOF
#define KEY_1_GPIO_Port GPIO_PIN_0
#define KEY_2_GPIO GPIOF
#define KEY_2_GPIO_Port GPIO_PIN_1
#define KEY_3_GPIO GPIOE
#define KEY_3_GPIO_Port GPIO_PIN_4
#define KEY_4_GPIO GPIOE
#define KEY_4_GPIO_Port GPIO_PIN_5
#define KEY_5_GPIO GPIOE
#define KEY_5_GPIO_Port GPIO_PIN_12


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
