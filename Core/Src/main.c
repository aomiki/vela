/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file                                                                       : main.c
 * @brief                                                                      : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "option/unicode.c"
#include "communication.h"
#include "flight_algorithm.h"
#include "gps.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define DEBOUNCE_DELAY 100

/* Калибровочные переменные */

/* Для температуры */
/// uint16_t dig_T1[1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t last_jumper_interrupt_time = 0;

uint32_t start_time; // Действительное время старта
uint32_t apogy_time; // Действительное время апогея с момента старта
uint32_t landing_time; // Действительное время приземления с момента старта

uint32_t time_to_apogee = 5000; // Расчетное время апогея с момента старта
uint32_t time_to_landing = 15000; // Расчетное время приземления с момента старта
uint32_t time_off = 5000; // Время от момента приземления до отключения

float apogy_height = 0.0f; // Расчетная высота апогея

bool do_read_sensors = false;
bool is_landing = false;
bool is_buzzer = false;
bool do_read_gps = false;
// const uint32_t start_height = 0;;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  
  Message msg = { .text = "⛵ Shellow from SSAU & Vela! ⛵\n\r\0", .sys_state = SYS_STATE_INIT, .sys_area = SYS_AREA_INIT, .priority = PRIORITY_HIGH };
	log_message(&msg);

  //(0) System initialization - start main algorithm
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	initialize_system();

  HAL_ADC_Start(&hadc1);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{
    //(1) Conditions for liftoff met - continue main algoirthm
    if (is_liftoff())
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      start_flight();
      start_time = HAL_GetTick(); // Millisecond

      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }

    //(2) Conditions for apogy met - continue main algorithm
    if (is_apogy())
    {
      apogy();
    }

    //Regular sensor read
    if (do_read_sensors)
    {
      read_sensors();
      do_read_sensors = false;
    }

    if (is_landing)
    {
      landing();
      is_landing = false;
    }

    if (is_buzzer)
    {
      buzz();
      is_buzzer = false;
    }

    if (do_read_gps)
    {
      GPS_UART_Callback();
      do_read_gps = false;
    }

/*
    // Таймер до приземления
    while (HAL_GetTick() - start_time < time_to_landing) {
      if (do_read_sensors) {
        do_read_sensors = false;
        read_sensors(); // Чтение датчиков, отправка и запись данных
      }
      if (check_landing()) {
        is_landing = true;
        break;
      }

      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      HAL_Delay(100);
    }

    landing_time = HAL_GetTick() - start_time; // Millisecond

    // Снизить период опроса датчиков и остановить отправку данных по радио
    HAL_TIM_Base_Stop_IT(&SENSORS_READ_TIM_HANDLE);
    HAL_TIM_Base_DeInit(&SENSORS_READ_TIM_HANDLE);

    SENSORS_READ_TIM_HANDLE.Init.Prescaler = 35999; // 72 MHz / 36000 = 2 кГц
    SENSORS_READ_TIM_HANDLE.Init.Period = 3999; // 4000 отсчётов = 2 сек

    HAL_TIM_Base_Init(&SENSORS_READ_TIM_HANDLE);
    HAL_TIM_Base_Start_IT(&SENSORS_READ_TIM_HANDLE);
*/

/*
    // Таймер до выключения платы // Опрос датчиков продолжается
    while (HAL_GetTick() - landing_time < time_off) {
      if (do_read_sensors) {
        do_read_sensors = false;
        read_sensors(); // Чтение датчиков, отправка и запись данных
      }

      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      HAL_Delay(200);
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      HAL_Delay(50);
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      HAL_Delay(200);
    }

    // Закончить работу с SD картой и остальными модулями
    // Начинает работу радио-маяк

    break;
*/
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    do_read_gps = true;
}

//INTERRUPT CALLBACKS

/// @brief EXT = External interrupts
/// @param GPIO_Pin Which pin interrupt came from
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == JUMPER_PIN) {
    uint32_t current_time = HAL_GetTick();

    // Проверка временного интервала для антидребезга
    if (current_time - last_jumper_interrupt_time > DEBOUNCE_DELAY) {
      last_jumper_interrupt_time = current_time;

      if (HAL_GPIO_ReadPin(JUMPER_PORT, JUMPER_PIN)) {
        _set_liftoff();
      }
    }
  }
  else {
		__NOP();
	}
}

/// @brief Timer interrupts
/// @param htim Which timer interrupt came from
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == SENSORS_READ_TIM_DEF) //check if the interrupt comes from TIM2
	{
		do_read_sensors = true;
	}

  if (htim->Instance == APOGY_TIM_DEF)
  {
    if (get_sys_state() == SYS_STATE_DESCENT)
    {
      is_landing = true;
    }
    else if (get_sys_state() == SYS_STATE_GROUND)
    {
      is_buzzer = true;
    }
    else
    {
      _set_apogy();
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex                                                                         : printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
