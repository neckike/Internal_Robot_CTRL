/*
 * tim.h
 *
 *  Created on: Dec 12, 2017
 *      Author: hector
 */

#ifndef __tim_H
#define __tim_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f3xx_hal.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM8_Init(void);
void MX_TIM15_Init(void);
void MX_TIM16_Init(void);
void MX_TIM17_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */
