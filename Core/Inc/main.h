/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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
#define FRONT_ISENSE_OUT_R_Pin GPIO_PIN_7
#define FRONT_ISENSE_OUT_R_GPIO_Port GPIOF
#define RIGHT_ENABLE_Pin GPIO_PIN_8
#define RIGHT_ENABLE_GPIO_Port GPIOF
#define LEFT_ENABLE_Pin GPIO_PIN_9
#define LEFT_ENABLE_GPIO_Port GPIOF
#define FRONT_ISENSE_OUT_L_Pin GPIO_PIN_10
#define FRONT_ISENSE_OUT_L_GPIO_Port GPIOF
#define ANLG_1_Pin GPIO_PIN_0
#define ANLG_1_GPIO_Port GPIOC
#define ANLG_2_Pin GPIO_PIN_1
#define ANLG_2_GPIO_Port GPIOC
#define ANLG_3_Pin GPIO_PIN_2
#define ANLG_3_GPIO_Port GPIOC
#define ANLG_5_Pin GPIO_PIN_3
#define ANLG_5_GPIO_Port GPIOC
#define DIG_1_Pin GPIO_PIN_3
#define DIG_1_GPIO_Port GPIOA
#define ANLG_4_Pin GPIO_PIN_4
#define ANLG_4_GPIO_Port GPIOA
#define DIG_2_Pin GPIO_PIN_6
#define DIG_2_GPIO_Port GPIOA
#define DRS_ISENSE_OUT_2_Pin GPIO_PIN_7
#define DRS_ISENSE_OUT_2_GPIO_Port GPIOA
#define DRS2_EN_Pin GPIO_PIN_4
#define DRS2_EN_GPIO_Port GPIOC
#define DRS1_EN_Pin GPIO_PIN_5
#define DRS1_EN_GPIO_Port GPIOC
#define DRS_ISENSE_OUT_1_Pin GPIO_PIN_0
#define DRS_ISENSE_OUT_1_GPIO_Port GPIOB
#define STATUS_1_Pin GPIO_PIN_1
#define STATUS_1_GPIO_Port GPIOB
#define STATUS2_Pin GPIO_PIN_2
#define STATUS2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
