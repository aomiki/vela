#include <stm32f411xe.h>
#include <tim.h>
#include <usart.h>

#ifndef __SYSTEM_DEFINITIONS_H
#define __SYSTEM_DEFINITIONS_H

#define SD_SPI_HANDLE hspi1
#define JUMPER_PORT GPIOB
#define JUMPER_PIN  GPIO_PIN_1

#define END_BUTTON_PORT GPIOA
#define END_BUTTON_PIN GPIO_PIN_4

//______________________________SENSORS
#define SENSORS_READ_TIM_DEF TIM2
#define SENSORS_READ_TIM_HANDLE htim2

//______________________________APOGY
#define APOGY_TIM_DEF TIM5
#define APOGY_TIM_HANDLE htim5

//______________________________SERVO
#define SERVO_TIM_DEF TIM3
#define SERVO_TIM_HANDLE htim3
#define SERVO_TIM_PWM_CHANNEL TIM_CHANNEL_2

//______________________________RADIO
#define RADIO_M0_PORT GPIOB
#define RADIO_M0_PIN GPIO_PIN_3
#define RADIO_M1_PORT GPIOA
#define RADIO_M1_PIN GPIO_PIN_15

#define RADIO_UART_HANDLE huart1

//______________________________GPS

#define GPS_UART_HANDLE huart2

//_____________________________LED
#define LED1_PORT GPIOB
#define LED1_PIN GPIO_PIN_14

#define LED2_PORT GPIOB
#define LED2_PIN GPIO_PIN_15

#define LED3_PORT GPIOB
#define LED3_PIN GPIO_PIN_13

#define LED4_PORT GPIOB
#define LED4_PIN GPIO_PIN_12

#define LED5_PORT GPIOA
#define LED5_PIN GPIO_PIN_8

#define LED6_PORT GPIOB
#define LED6_PIN GPIO_PIN_4

#endif
