#ifndef __SensorDataHandler_H
#define __SensorDataHandler_H
#ifdef __cplusplus
extern "C"
{
#endif
#include "stm32f3xx_hal.h"
#include "main.h"
#include "internal_sensors.h"
#include "ultrasonic.h"
#include "ControllerAreaNetwork.h"
#include "queue.h"
//extern sensorPin right_sensor;
//extern sensorPin back_sensor;
//extern sensorPin left_sensor;
//extern sensorPin front_sensor;

#define SAMPLE_TIME_DEFAULT	50
#define UNITS_TIME_DEFAULT 0	//miliseconds default

typedef enum {
	NO_TRANSMISSION,
	PUBLISHER,
	ONE_TIME_TRANSMISSION,
} cmhandler_mode;

typedef enum {
	MILISECONDS,
	SECONDS,
	MINUTES,
	MICROSECONDS,
} unit_publisher;

typedef enum {
	GENERAL_WARNING,
	GENERAL_ERROR,
	MODE1_TIMEOUT,
	LOAD_CONFIGURATION_TIMEOUT,
} cmhandler_component_status;

QueueHandle_t CM_Handler_Queue;

void initHardware(void);
//void handleACommand(void);

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */
