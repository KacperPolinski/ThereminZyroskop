/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32l4xx_hal.h"

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
#define SAI1_MCK_Pin GPIO_PIN_2
#define SAI1_MCK_GPIO_Port GPIOE
#define AUDIO_RST_Pin GPIO_PIN_3
#define AUDIO_RST_GPIO_Port GPIOE
#define SAI1_FS_Pin GPIO_PIN_4
#define SAI1_FS_GPIO_Port GPIOE
#define SAI1_SCK_Pin GPIO_PIN_5
#define SAI1_SCK_GPIO_Port GPIOE
#define SAI1_SD_Pin GPIO_PIN_6
#define SAI1_SD_GPIO_Port GPIOE
#define MAG_CS_Pin GPIO_PIN_0
#define MAG_CS_GPIO_Port GPIOC
#define MEMS_SCK_Pin GPIO_PIN_1
#define MEMS_SCK_GPIO_Port GPIOD
#define MEMS_MOSI_Pin GPIO_PIN_4
#define MEMS_MOSI_GPIO_Port GPIOD
#define GYRO_CS_Pin GPIO_PIN_7
#define GYRO_CS_GPIO_Port GPIOD
#define M3V3_REG_ON_Pin GPIO_PIN_3
#define M3V3_REG_ON_GPIO_Port GPIOB
#define XL_CS_Pin GPIO_PIN_0
#define XL_CS_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
