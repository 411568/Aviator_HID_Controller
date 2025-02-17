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
#include "stm32f0xx_hal.h"

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
#define Button0_Pin GPIO_PIN_13
#define Button0_GPIO_Port GPIOC
#define Button1_Pin GPIO_PIN_14
#define Button1_GPIO_Port GPIOC
#define Button2_Pin GPIO_PIN_15
#define Button2_GPIO_Port GPIOC
#define EncB1_Pin GPIO_PIN_0
#define EncB1_GPIO_Port GPIOA
#define EncA1_Pin GPIO_PIN_1
#define EncA1_GPIO_Port GPIOA
#define Button3_Pin GPIO_PIN_3
#define Button3_GPIO_Port GPIOA
#define Button4_Pin GPIO_PIN_4
#define Button4_GPIO_Port GPIOA
#define Button5_Pin GPIO_PIN_5
#define Button5_GPIO_Port GPIOA
#define EncB2_Pin GPIO_PIN_6
#define EncB2_GPIO_Port GPIOA
#define EncA2_Pin GPIO_PIN_7
#define EncA2_GPIO_Port GPIOA
#define Button6_Pin GPIO_PIN_0
#define Button6_GPIO_Port GPIOB
#define Button7_Pin GPIO_PIN_1
#define Button7_GPIO_Port GPIOB
#define Button8_Pin GPIO_PIN_2
#define Button8_GPIO_Port GPIOB
#define Button9_Pin GPIO_PIN_10
#define Button9_GPIO_Port GPIOB
#define Button10_Pin GPIO_PIN_11
#define Button10_GPIO_Port GPIOB
#define Button11_Pin GPIO_PIN_12
#define Button11_GPIO_Port GPIOB
#define EncB0_Pin GPIO_PIN_8
#define EncB0_GPIO_Port GPIOA
#define EncA0_Pin GPIO_PIN_9
#define EncA0_GPIO_Port GPIOA
#define out1_Pin GPIO_PIN_8
#define out1_GPIO_Port GPIOB
#define out2_Pin GPIO_PIN_9
#define out2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
