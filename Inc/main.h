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
#include "stm32g4xx_hal.h"

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
[[noreturn]] void app();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIAG0_Pin GPIO_PIN_2
#define DIAG0_GPIO_Port GPIOC
#define DIAG1_Pin GPIO_PIN_3
#define DIAG1_GPIO_Port GPIOC
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define DRV_EN_Pin GPIO_PIN_5
#define DRV_EN_GPIO_Port GPIOC
#define CFG4_Pin GPIO_PIN_0
#define CFG4_GPIO_Port GPIOB
#define CFG5_Pin GPIO_PIN_1
#define CFG5_GPIO_Port GPIOB
#define CFG6_Pin GPIO_PIN_2
#define CFG6_GPIO_Port GPIOB
#define DIP_SW1_Pin GPIO_PIN_10
#define DIP_SW1_GPIO_Port GPIOB
#define DIP_SW2_Pin GPIO_PIN_11
#define DIP_SW2_GPIO_Port GPIOB
#define DIP_SW3_Pin GPIO_PIN_12
#define DIP_SW3_GPIO_Port GPIOB
#define DIP_SW4_Pin GPIO_PIN_13
#define DIP_SW4_GPIO_Port GPIOB
#define DIP_SW5_Pin GPIO_PIN_14
#define DIP_SW5_GPIO_Port GPIOB
#define DIP_SW6_Pin GPIO_PIN_15
#define DIP_SW6_GPIO_Port GPIOB
#define DIP_SW7_Pin GPIO_PIN_8
#define DIP_SW7_GPIO_Port GPIOC
#define DIP_SW8_Pin GPIO_PIN_9
#define DIP_SW8_GPIO_Port GPIOC
#define REFL_STEP_Pin GPIO_PIN_8
#define REFL_STEP_GPIO_Port GPIOA
#define REFL_DIR_Pin GPIO_PIN_9
#define REFL_DIR_GPIO_Port GPIOA
#define SPI_MODE_Pin GPIO_PIN_11
#define SPI_MODE_GPIO_Port GPIOA
#define SD_MODE_Pin GPIO_PIN_12
#define SD_MODE_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
