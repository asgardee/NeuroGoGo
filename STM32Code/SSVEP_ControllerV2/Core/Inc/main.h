/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#define SPIComp_NSS_Pin GPIO_PIN_4
#define SPIComp_NSS_GPIO_Port GPIOA
#define SPIComp_SCK_Pin GPIO_PIN_5
#define SPIComp_SCK_GPIO_Port GPIOA
#define SPIComp_MISO_Pin GPIO_PIN_6
#define SPIComp_MISO_GPIO_Port GPIOA
#define SPIComp_MOSI_Pin GPIO_PIN_7
#define SPIComp_MOSI_GPIO_Port GPIOA
#define RightTurn_Pin GPIO_PIN_0
#define RightTurn_GPIO_Port GPIOB
#define LeftTurn_Pin GPIO_PIN_1
#define LeftTurn_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_3
#define LED4_GPIO_Port GPIOB
#define Move3_Pin GPIO_PIN_4
#define Move3_GPIO_Port GPIOB
#define Move4_Pin GPIO_PIN_5
#define Move4_GPIO_Port GPIOB
#define GyroSCL_Pin GPIO_PIN_6
#define GyroSCL_GPIO_Port GPIOB
#define GyroSDA_Pin GPIO_PIN_7
#define GyroSDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/