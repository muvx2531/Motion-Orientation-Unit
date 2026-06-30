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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MAG_INT_Pin GPIO_PIN_0
#define MAG_INT_GPIO_Port GPIOF
#define SPI_MAG_CS_Pin GPIO_PIN_0
#define SPI_MAG_CS_GPIO_Port GPIOA
#define SPI_PRS_CS_Pin GPIO_PIN_1
#define SPI_PRS_CS_GPIO_Port GPIOA
#define SPI_IMU_CS_Pin GPIO_PIN_4
#define SPI_IMU_CS_GPIO_Port GPIOA
#define IMU_FSync_Pin GPIO_PIN_0
#define IMU_FSync_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_10
#define IMU_INT_GPIO_Port GPIOA
#define LED_Blink_Pin GPIO_PIN_4
#define LED_Blink_GPIO_Port GPIOB
#define GPS_PowerON_Pin GPIO_PIN_6
#define GPS_PowerON_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
