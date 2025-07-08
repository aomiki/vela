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
#define led_internal_Pin GPIO_PIN_13
#define led_internal_GPIO_Port GPIOC
#define jumper_Pin GPIO_PIN_1
#define jumper_GPIO_Port GPIOA
#define jumper_EXTI_IRQn EXTI1_IRQn
#define gps_uart_tx_Pin GPIO_PIN_2
#define gps_uart_tx_GPIO_Port GPIOA
#define gps_uart_rx_Pin GPIO_PIN_3
#define gps_uart_rx_GPIO_Port GPIOA
#define limit_switch_Pin GPIO_PIN_4
#define limit_switch_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_1
#define SD_CS_GPIO_Port GPIOB
#define vent_Pin GPIO_PIN_10
#define vent_GPIO_Port GPIOB
#define led_acc_Pin GPIO_PIN_12
#define led_acc_GPIO_Port GPIOB
#define led_sd_Pin GPIO_PIN_13
#define led_sd_GPIO_Port GPIOB
#define led_servo_Pin GPIO_PIN_14
#define led_servo_GPIO_Port GPIOB
#define led_radio_Pin GPIO_PIN_15
#define led_radio_GPIO_Port GPIOB
#define led_bar_Pin GPIO_PIN_8
#define led_bar_GPIO_Port GPIOA
#define radio_uart_tx_Pin GPIO_PIN_9
#define radio_uart_tx_GPIO_Port GPIOA
#define radio_uart_rx_Pin GPIO_PIN_10
#define radio_uart_rx_GPIO_Port GPIOA
#define radio_aux_Pin GPIO_PIN_11
#define radio_aux_GPIO_Port GPIOA
#define radio_m1_Pin GPIO_PIN_15
#define radio_m1_GPIO_Port GPIOA
#define radio_m0_Pin GPIO_PIN_3
#define radio_m0_GPIO_Port GPIOB
#define led_gps_Pin GPIO_PIN_4
#define led_gps_GPIO_Port GPIOB
#define buzzer_tim4_ch3_Pin GPIO_PIN_8
#define buzzer_tim4_ch3_GPIO_Port GPIOB
#define beacon_aux_Pin GPIO_PIN_9
#define beacon_aux_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#include "system_definitions.h"

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
