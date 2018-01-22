#ifndef __internal_sensors_H
#define __internal_sensors_H
#ifdef __cplusplus
extern "C"
{
#endif
#include "stm32f3xx_hal.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

float gyro[3];
float accel[3];
float magn[3];

void MX_I2C1_Init(void);
void MX_SPI1_Init(void);
void initGyroscope(void);
void initAccelerometer(void);
void initMagnetometer(void);
uint8_t readGyroscope(float* data);
uint8_t readAccelerometer(float* data);
uint8_t readMagnetometer(float* data);


#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */
