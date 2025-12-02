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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MA.h"

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
#define SD_ADC_Pin GPIO_PIN_0
#define SD_ADC_GPIO_Port GPIOA
#define GPIO1_Pin GPIO_PIN_1
#define GPIO1_GPIO_Port GPIOA
#define GPIO2_Pin GPIO_PIN_2
#define GPIO2_GPIO_Port GPIOA
#define GPIO3_Pin GPIO_PIN_3
#define GPIO3_GPIO_Port GPIOA
#define SD_DAC_Pin GPIO_PIN_4
#define SD_DAC_GPIO_Port GPIOA
#define GPIO5_Pin GPIO_PIN_5
#define GPIO5_GPIO_Port GPIOA
#define GPIO6_Pin GPIO_PIN_6
#define GPIO6_GPIO_Port GPIOA
#define GPIO7_Pin GPIO_PIN_7
#define GPIO7_GPIO_Port GPIOA
#define GPIO0_Pin GPIO_PIN_0
#define GPIO0_GPIO_Port GPIOB
#define GPIO8_Pin GPIO_PIN_8
#define GPIO8_GPIO_Port GPIOA
#define GPIO9_Pin GPIO_PIN_11
#define GPIO9_GPIO_Port GPIOA
#define SD_CLK_Pin GPIO_PIN_3
#define SD_CLK_GPIO_Port GPIOB
#define SD_CLK_EXTI_IRQn EXTI3_IRQn
#define GPIO4_Pin GPIO_PIN_4
#define GPIO4_GPIO_Port GPIOB
#define SD_CNT_Pin GPIO_PIN_5
#define SD_CNT_GPIO_Port GPIOB
#define SD_DOUT_Pin GPIO_PIN_6
#define SD_DOUT_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_7
#define LED_B_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_8
#define LED_G_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
