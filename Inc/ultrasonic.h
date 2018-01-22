#ifndef __ultrasonic_H
#define __ultrasonic_H
#ifdef __cplusplus
extern "C"
{
#endif
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "stm32f3xx_hal_def.h"

#define HCSR04_TIMEOUT			1000000
#define HCSR04_NUMBER			((float)0.0171821)


typedef struct sensorPin{
	uint16_t	num_trig;
	uint16_t	num_echo;
	uint32_t	grp_trig;
	uint32_t	grp_echo;
}sensorPin;


float readUltrasonic(sensorPin *sensor);


#ifdef __cplusplus
}
#endif
#endif
