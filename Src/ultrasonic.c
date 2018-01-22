/*
 * ultrasonic.c
 *
 *  Created on: Dec 14, 2017
 *      Author: hector
 */
#include "ultrasonic.h"
const TickType_t timeDelayUltras = 1 / (portTICK_PERIOD_MS*1000);	//1us

float readUltrasonic(sensorPin *sensor){

	uint32_t time, timeout;
	HAL_GPIO_WritePin(sensor->grp_trig, sensor->num_trig, GPIO_PIN_RESET);
	vTaskDelay(timeDelayUltras*2);	//2us
	HAL_GPIO_WritePin(sensor->grp_trig, sensor->num_trig, GPIO_PIN_SET);
	vTaskDelay(timeDelayUltras*10);	//10us
	HAL_GPIO_WritePin(sensor->grp_trig, sensor->num_trig, GPIO_PIN_RESET);
	timeout = HCSR04_TIMEOUT;

	while (!HAL_GPIO_ReadPin(sensor->grp_echo, sensor->num_echo)) {
		if (timeout-- == 0x00) {
			//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);		//LED3_RED		NORTH
			return -1;
		}
	}
	time = 0;
	while (HAL_GPIO_ReadPin(sensor->grp_echo, sensor->num_echo)) {
		time++;
		/* Delay 1us */
		vTaskDelay(timeDelayUltras);
	}
	/* Convert us to cm */
	//distance =  (float)time * HCSR04_NUMBER;
	return (float)time /58 - 20;
}
