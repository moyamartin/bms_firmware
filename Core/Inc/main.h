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
#define SWT_GPIO_Pin GPIO_PIN_2
#define SWT_GPIO_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define BQ24_PG_Pin GPIO_PIN_0
#define BQ24_PG_GPIO_Port GPIOC
#define BQ24_STAT2_Pin GPIO_PIN_1
#define BQ24_STAT2_GPIO_Port GPIOC
#define BQ24_STAT1_Pin GPIO_PIN_2
#define BQ24_STAT1_GPIO_Port GPIOC
#define BQ24_CE_Pin GPIO_PIN_3
#define BQ24_CE_GPIO_Port GPIOC
#define BQ76_CS_Pin GPIO_PIN_4
#define BQ76_CS_GPIO_Port GPIOA
#define BQ76_FAULT_Pin GPIO_PIN_4
#define BQ76_FAULT_GPIO_Port GPIOC
#define BQ76_FAULT_EXTI_IRQn EXTI4_IRQn
#define BQ76_ALERT_Pin GPIO_PIN_5
#define BQ76_ALERT_GPIO_Port GPIOC
#define BQ76_ALERT_EXTI_IRQn EXTI9_5_IRQn
#define BQ76_DRDY_Pin GPIO_PIN_0
#define BQ76_DRDY_GPIO_Port GPIOB
#define BQ76_DRDY_EXTI_IRQn EXTI0_IRQn
#define BQ76_CONV_Pin GPIO_PIN_1
#define BQ76_CONV_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define UART5_TX_Pin GPIO_PIN_12
#define UART5_TX_GPIO_Port GPIOC
#define UART5_RX_Pin GPIO_PIN_2
#define UART5_RX_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
