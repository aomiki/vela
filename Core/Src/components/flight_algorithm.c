#include "flight_algorithm.h"
#include "system_definitions.h"
#include "communication.h"
#include "barometer.h"
#include "accelerometer.h"
#include "tim.h"
#include "sd_card.h"
#include "radio.h"
#include "servo.h"
#include "adc.h"
#include "gps.h"
#include <stdio.h>
#include <math.h>

#define SEA_LEVEL_PRESSURE 101325.0f // Стандартное давление на уровне моря (Па)
#define GRAVITY 9.81f // Ускорение свободного падения (м/с²)
#define GAS_CONSTANT 287.05f // Удельная газовая постоянная для воздуха (Дж/(кг·K))
#define LAPSE_RATE 0.0065f // Градиент температуры (K/m)

// float start_height = 0.0f;
uint32_t previousValue = 0;
const uint32_t threshold = 200;  // Порог изменения (нужно подбирать)

static Peripheral enabled_peripheral = 0;

static SystemState curr_sys_state = SYS_STATE_NONE;
static float start_altitude = 0;

SystemState get_sys_state()
{
	return curr_sys_state;
}

void change_enabled_periph_bit(bool is_enabled, Peripheral periph_bit)
{
	if (is_enabled)
	{
		enabled_peripheral |= periph_bit;
	}
	else
	{
		enabled_peripheral &= !periph_bit;
	}
}

void switch_read_sensors_ground()
{
	//Reduce frequence of telemetry polling
	HAL_TIM_Base_Stop_IT(&SENSORS_READ_TIM_HANDLE);
	
	__HAL_TIM_SET_PRESCALER(&SENSORS_READ_TIM_HANDLE, 0);
	HAL_TIM_GenerateEvent(&SENSORS_READ_TIM_HANDLE, TIM_EVENTSOURCE_UPDATE); // so the new prescaler is loaded
	__HAL_TIM_CLEAR_FLAG(&SENSORS_READ_TIM_HANDLE, TIM_FLAG_UPDATE); // so it doesn't run right away

	__HAL_TIM_SET_AUTORELOAD(&SENSORS_READ_TIM_HANDLE, 2160000000);

	HAL_TIM_Base_Start_IT(&SENSORS_READ_TIM_HANDLE);
}

void switch_read_sensors_standby()
{
	//Reduce frequence of telemetry polling
	HAL_TIM_Base_Stop_IT(&SENSORS_READ_TIM_HANDLE);

	__HAL_TIM_SET_PRESCALER(&SENSORS_READ_TIM_HANDLE, 0);
	HAL_TIM_GenerateEvent(&SENSORS_READ_TIM_HANDLE, TIM_EVENTSOURCE_UPDATE); // so the new prescaler is loaded
	__HAL_TIM_CLEAR_FLAG(&SENSORS_READ_TIM_HANDLE, TIM_FLAG_UPDATE); // so it doesn't run right away

	__HAL_TIM_SET_AUTORELOAD(&SENSORS_READ_TIM_HANDLE, 720000000);

	HAL_TIM_Base_Start_IT(&SENSORS_READ_TIM_HANDLE);
}

void switch_read_sensors_flight()
{
	//Reduce frequence of telemetry polling
	HAL_TIM_Base_Stop_IT(&SENSORS_READ_TIM_HANDLE);

	__HAL_TIM_SET_PRESCALER(&SENSORS_READ_TIM_HANDLE, 0);
	HAL_TIM_GenerateEvent(&SENSORS_READ_TIM_HANDLE, TIM_EVENTSOURCE_UPDATE); // so the new prescaler is loaded
	__HAL_TIM_CLEAR_FLAG(&SENSORS_READ_TIM_HANDLE, TIM_FLAG_UPDATE); // so it doesn't run right away

	__HAL_TIM_SET_AUTORELOAD(&SENSORS_READ_TIM_HANDLE, 72000000);

	HAL_TIM_Base_Start_IT(&SENSORS_READ_TIM_HANDLE);
}

void switch_apogy_tim_descent()
{
	HAL_TIM_Base_Stop_IT(&APOGY_TIM_HANDLE);

	__HAL_TIM_SET_PRESCALER(&APOGY_TIM_HANDLE, 0);
	HAL_TIM_GenerateEvent(&APOGY_TIM_HANDLE, TIM_EVENTSOURCE_UPDATE); // so the new prescaler is loaded
	__HAL_TIM_CLEAR_FLAG(&APOGY_TIM_HANDLE, TIM_FLAG_UPDATE); // so it doesn't run right away

	__HAL_TIM_SET_AUTORELOAD(&APOGY_TIM_HANDLE, 1584000000);

	HAL_TIM_Base_Start_IT(&APOGY_TIM_HANDLE);
}

void update_enabled_peripheral()
{
	change_enabled_periph_bit(sd_card_is_enabled(), PERIPH_SD);
	change_enabled_periph_bit(radio_is_enabled(), PERIPH_RADIO);
	change_enabled_periph_bit(HAL_GPIO_ReadPin(JUMPER_PORT, JUMPER_PIN), PERIPH_JUMPER);
	change_enabled_periph_bit(check_acc_identity(), PERIPH_ACC);
	change_enabled_periph_bit(check_barometer_identity(), PERIPH_BAROM);
}

float get_altitude(float pressure, float temperature)
{	
	if(pressure <= 0 || pressure > SEA_LEVEL_PRESSURE * 1.5f) {
		return -9999.0f;  // Некорректное давление
	}

	float temp_kelvin = temperature + 273.15f;

	float height = (temp_kelvin / LAPSE_RATE) * (
		1.0f - powf(
			pressure / SEA_LEVEL_PRESSURE, 
			(GAS_CONSTANT * LAPSE_RATE) / GRAVITY
		)
	);

	return height;
}

void read_telemetry(Telemetry* tel)
{
	//BAROMETER
	if (enabled_peripheral & PERIPH_BAROM)
	{
		tel->temp = ((float)read_temp()) / 100;
		tel->pressure = ((float)read_pressure()) / 256;
		tel->altitude = get_altitude(tel->pressure, tel->temp) - start_altitude;
	}

	//ACCELEROMETER
	if (enabled_peripheral & PERIPH_ACC)
	{
		double acc_vals[3];
		read_acceleration_xyz(acc_vals);

		tel->acc_x = acc_vals[0];
		tel->acc_y = acc_vals[1];
		tel->acc_z = acc_vals[2];
	}

	if (enabled_peripheral & PERIPH_GPS)
	{
		tel->gps = *GPS_GetData();
	}
}

void landing()
{
	curr_sys_state = SYS_STATE_GROUND;

	Telemetry tel;
	set_default_telemetry(&tel);
	tel.sys_area = SYS_AREA_READ_SENSORS;
	tel.sys_status = enabled_peripheral;
	tel.sys_state = get_sys_state();
	log_telemetry(&tel);

	//Stop apogy timer
	HAL_TIM_Base_Stop_IT(&APOGY_TIM_HANDLE);

	switch_read_sensors_ground();
}

void read_sensors()
{
	Telemetry tel;
	set_default_telemetry(&tel);
	tel.sys_area = SYS_AREA_READ_SENSORS;
	tel.sys_status = enabled_peripheral;
	tel.sys_state = get_sys_state();

	update_enabled_peripheral(); //maybe some peripheral died / came back from the dead?

	read_telemetry(&tel);

	log_telemetry(&tel);
}

bool check_rescue() {
	// Проверяем концевую кнопку
	if (HAL_GPIO_ReadPin(END_BUTTON_PORT, END_BUTTON_PIN) == GPIO_PIN_RESET) {
		return 1;
	}

	// Проверяем фоторезистор
	/* uint32_t currentValue = HAL_ADC_GetValue(&hadc1);
	int32_t difference = currentValue - previousValue;

	if (difference > threshold) {
	  // Резкое осветление
	  return 1;
	}
	previousValue = currentValue; */

	return 0;
}

bool check_apogy() {
	// Проверяем высоту (get_altitude())
	

	// Проверяем акселерометр


	return 0;
}

void open_rescue() {
	servo_turn_apogy();
	HAL_Delay(1000); //
	servo_turn_max();
}

bool check_landing() {
	// Проверяем высоту (get_altitude())


	// Проверяем акселерометр
	

	return 0;
}

void apogy()
{
	//9.96 seconds until apogy
	curr_sys_state = SYS_STATE_APOGY;

	HAL_TIM_Base_Stop_IT(&APOGY_TIM_HANDLE);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	Message msg = { .sys_area = SYS_AREA_MAIN_ALGO, .sys_state = SYS_STATE_APOGY, .priority = PRIORITY_HIGH };
	msg.text = malloc(256);
	
	sprintf(msg.text, "Apogy! Initiating rescue.\r\n");
	log_message(&msg);

	//Just don't think, fire it multiple times
	for (size_t i = 0; i < 3; i++)
	{
		open_rescue();

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_Delay(300);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	}

	//Hope that it works
	if (check_rescue())
	{
		sprintf(msg.text, "Rescue system open!\r\n");
		log_message(&msg);
	}
	else
	{
		sprintf(msg.text, "Rescue system failed to open! Retrying...\r\n");
		log_message(&msg);

		//Try a bit more
		for (size_t i = 0; i < 5; i++)
		{
			open_rescue();

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_Delay(300);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		}

		if (check_rescue())
		{
			sprintf(msg.text, "Rescue system finally opened!\r\n");
			log_message(&msg);
		}
		else
		{
			sprintf(msg.text, "Rescue system failed to open anyway.\r\n");
			log_message(&msg);
		}
	}

	free(msg.text);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	Telemetry tel;
	set_default_telemetry(&tel);
	tel.sys_state = get_sys_state();
	tel.sys_status = enabled_peripheral;
	tel.sys_area = SYS_AREA_MAIN_ALGO;

	log_telemetry(&tel);

	curr_sys_state = SYS_STATE_DESCENT;

	switch_apogy_tim_descent();
}

void start_flight()
{
	curr_sys_state = SYS_STATE_LIFTOFF;

	switch_read_sensors_flight();

	/*
	//start reading sensors
	__HAL_TIM_SET_COUNTER(&SENSORS_READ_TIM_HANDLE, 0);
	__HAL_TIM_CLEAR_FLAG(&SENSORS_READ_TIM_HANDLE, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(&SENSORS_READ_TIM_HANDLE);
	*/

	//count down ot apogy
	__HAL_TIM_SET_COUNTER(&APOGY_TIM_HANDLE, 0);
	__HAL_TIM_CLEAR_FLAG(&APOGY_TIM_HANDLE, TIM_FLAG_UPDATE);

	HAL_TIM_Base_Start_IT(&APOGY_TIM_HANDLE);

	send_status(0x0);
	Telemetry tel;
	set_default_telemetry(&tel);
	tel.sys_state = get_sys_state();
	tel.sys_status = enabled_peripheral;
	tel.sys_area = SYS_AREA_MAIN_ALGO;

	log_telemetry(&tel);
	Message msg = { .text = "🚀 Поплыли к звездам! 🚀 \n\n\n\r\0", .sys_area = SYS_AREA_MAIN_ALGO, .sys_state = curr_sys_state, .priority = PRIORITY_HIGH };
	log_message(&msg);

	curr_sys_state = SYS_STATE_ASCENT;
}

void initialize_system()
{
	curr_sys_state = SYS_STATE_INIT;

	// Start fan
	HAL_GPIO_WritePin(vent_GPIO_Port, vent_Pin, GPIO_PIN_SET);

	Message msg = { .sys_area = SYS_AREA_INIT, .sys_state = SYS_STATE_INIT, .priority = PRIORITY_HIGH };
	msg.text = malloc(256);

	sprintf(msg.text, "_____________ [begin system init] \n\r");
	log_message(&msg);

	enabled_peripheral = 0;

	HAL_Delay(1000); //Let everything power on

	//0. Radio
	msg.priority = PRIORITY_HIGH;
	msg.sys_area = SYS_AREA_PERIPH_RADIO;
	sprintf(msg.text, "[init: radio]_____\n\r");
	log_message(&msg);

	radio_init();
	enabled_peripheral |= PERIPH_RADIO;

	//1. SD CARD.
	msg.priority = PRIORITY_LOW;
	msg.sys_area = SYS_AREA_PERIPH_SDCARD;
	sprintf(msg.text, "_____[init: sd card]\n\r");
	log_message(&msg);

	sd_status sd_stat = sd_card_mount();

	if (sd_stat == SD_OK)
	{
		msg.priority = PRIORITY_MEDIUM;
		sprintf(msg.text, "sd card mounted\r\n");
		log_message(&msg);

		sd_file file;
		sd_stat = sd_card_open_file(&file, "/test");

		if (sd_stat == SD_OK)
		{
			msg.priority = PRIORITY_MEDIUM;
			sprintf(msg.text, "sd card test file opened\r\n");
			log_message(&msg);

			sd_stat = sd_card_write(&file, "Good luck, good flight.\r\n");

			if (sd_stat == SD_OK)
			{
				msg.priority = PRIORITY_HIGH;
				sprintf(msg.text, "sd card test file written, sd works\r\n");
				log_message(&msg);
			}
			else
			{
				msg.priority = PRIORITY_HIGH;
				sprintf(msg.text, "sd card test file write failure!\r\n");
				log_message(&msg);
			}

			sd_card_close(&file);
		}
		else
		{
			msg.priority = PRIORITY_HIGH;
			sprintf(msg.text, "sd card test file failed to open!\r\n");
			log_message(&msg);
		}

		_sd_card_set_enabled();
		enabled_peripheral |= PERIPH_SD;
	}
	else
	{
		msg.priority = PRIORITY_HIGH;
		sprintf(msg.text, "sd card failed to mount!\r\n");
		log_message(&msg);
	}

	if(GPS_Init())
	{
		enabled_peripheral |= PERIPH_GPS;
	}

	//2. ACCELEROMETER
	msg.priority = PRIORITY_LOW;
	msg.sys_area = SYS_AREA_PERIPH_ACC;
	sprintf(msg.text, "[init: acc]_____\r\n");
	log_message(&msg);

	if (check_acc_identity())
	{
		msg.priority = PRIORITY_HIGH;
		sprintf(msg.text, "accelerometer responds nicely, powering it on...\r\n");
		log_message(&msg);

		acc_power_on();
		enabled_peripheral |= PERIPH_ACC;
	}
	else
	{
		msg.priority = PRIORITY_HIGH;
		sprintf(msg.text, "!!!accelerometer not responding!!!\n\r");
		log_message(&msg);
	}

	//3. BAROMETER
	msg.sys_area = SYS_AREA_PERIPH_BAROM;
	msg.priority = PRIORITY_LOW;
	sprintf(msg.text, "[init: barometer]_____\n\r");
	log_message(&msg);

	if (check_barometer_identity())
	{
		msg.priority = PRIORITY_HIGH;
		sprintf(msg.text, "barometer responds correctly, powering on...\n\r");
		log_message(&msg);

		barometer_power_on();

		enabled_peripheral |= PERIPH_BAROM;
	}
	else
	{
		msg.priority = PRIORITY_HIGH;
		sprintf(msg.text, "barometer not responding!\n\r");
		log_message(&msg);
	}

	//4. SERVO
	if(HAL_TIM_PWM_Start(&SERVO_TIM_HANDLE, SERVO_TIM_PWM_CHANNEL) == HAL_OK)
	{
		enabled_peripheral |= PERIPH_SERVO;
		//servo_turn_min();
	}

	//5. JUMPER
	if (HAL_GPIO_ReadPin(JUMPER_PORT, JUMPER_PIN))
	{
		enabled_peripheral |= PERIPH_JUMPER;
	}

	send_status(enabled_peripheral);

	HAL_Delay(100); // Let everything initialize properly

	//Do first read of telemetry, set initial values, talk to outside.
	Telemetry tel;
	set_default_telemetry(&tel);
	tel.sys_status = enabled_peripheral;
	tel.sys_state = get_sys_state();
	tel.sys_area = SYS_STATE_INIT;

	read_telemetry(&tel);

	start_altitude = tel.altitude;

	log_telemetry(&tel);

	msg.sys_state = SYS_AREA_INIT;
	msg.priority = PRIORITY_HIGH;
	sprintf(msg.text, "[end system init]_____________\n\r");
	log_message(&msg);

	free(msg.text);

	curr_sys_state = SYS_STATE_STANDBY;

	switch_read_sensors_standby();
}
