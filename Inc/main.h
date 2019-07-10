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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FET8_Pin GPIO_PIN_2
#define FET8_GPIO_Port GPIOE
#define FET7_Pin GPIO_PIN_3
#define FET7_GPIO_Port GPIOE
#define FET6_Pin GPIO_PIN_4
#define FET6_GPIO_Port GPIOE
#define FET5_Pin GPIO_PIN_5
#define FET5_GPIO_Port GPIOE
#define FET4_Pin GPIO_PIN_6
#define FET4_GPIO_Port GPIOE
#define FET3_Pin GPIO_PIN_13
#define FET3_GPIO_Port GPIOC
#define FET2_Pin GPIO_PIN_14
#define FET2_GPIO_Port GPIOC
#define FET1_Pin GPIO_PIN_15
#define FET1_GPIO_Port GPIOC
#define EMERGENCY_Pin GPIO_PIN_0
#define EMERGENCY_GPIO_Port GPIOC
#define ADC4_Pin GPIO_PIN_2
#define ADC4_GPIO_Port GPIOC
#define ADC3_Pin GPIO_PIN_3
#define ADC3_GPIO_Port GPIOC
#define LIPO_Pin GPIO_PIN_3
#define LIPO_GPIO_Port GPIOA
#define RO2_A_Pin GPIO_PIN_5
#define RO2_A_GPIO_Port GPIOA
#define ADC2_Pin GPIO_PIN_0
#define ADC2_GPIO_Port GPIOB
#define ADC1_Pin GPIO_PIN_1
#define ADC1_GPIO_Port GPIOB
#define USB_OTG_HS_VBUS_Pin GPIO_PIN_8
#define USB_OTG_HS_VBUS_GPIO_Port GPIOD
#define HUMAN1_Pin GPIO_PIN_9
#define HUMAN1_GPIO_Port GPIOD
#define HUMAN2_Pin GPIO_PIN_10
#define HUMAN2_GPIO_Port GPIOD
#define HUMAN3_Pin GPIO_PIN_11
#define HUMAN3_GPIO_Port GPIOD
#define BUZZER1_Pin GPIO_PIN_12
#define BUZZER1_GPIO_Port GPIOD
#define BUZZER2_Pin GPIO_PIN_13
#define BUZZER2_GPIO_Port GPIOD
#define RO1_A_Pin GPIO_PIN_6
#define RO1_A_GPIO_Port GPIOC
#define RO1_B_Pin GPIO_PIN_7
#define RO1_B_GPIO_Port GPIOC
#define MPU_SS_Pin GPIO_PIN_0
#define MPU_SS_GPIO_Port GPIOD
#define SPI_R_SS_Pin GPIO_PIN_1
#define SPI_R_SS_GPIO_Port GPIOD
#define RS_SIG2_Pin GPIO_PIN_2
#define RS_SIG2_GPIO_Port GPIOD
#define RS_SIG1_Pin GPIO_PIN_3
#define RS_SIG1_GPIO_Port GPIOD
#define LTC_SIG_Pin GPIO_PIN_4
#define LTC_SIG_GPIO_Port GPIOD
#define RO2_B_Pin GPIO_PIN_3
#define RO2_B_GPIO_Port GPIOB
#define PING_Pin GPIO_PIN_7
#define PING_GPIO_Port GPIOB
#define RS_SIG2B8_Pin GPIO_PIN_8
#define RS_SIG2B8_GPIO_Port GPIOB
#define RS_SIG1B9_Pin GPIO_PIN_9
#define RS_SIG1B9_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
