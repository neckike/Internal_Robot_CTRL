#include "cmsis_os.h"
#include "gpio.h"
#include "internal_sensors.h"

float mdps_per_digit = 8.75;

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

void initGyroscope(void){
	uint8_t data_ctrl1, address_ctrl1;

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	address_ctrl1 = 0x20;
	HAL_SPI_Transmit(&hspi1, &address_ctrl1, 1, 50);
	data_ctrl1 = 0xcf;
	HAL_SPI_Transmit(&hspi1, &data_ctrl1, 1, 50);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}
void initAccelerometer(void){
	uint8_t accel_address = 0x32;
	uint8_t accel_reg, accel_val;


	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	accel_reg=0x20;
	accel_val=0x57;		//0x67 for 100Hz	0x97 for 800Hz (as before)
	uint8_t cmd[] = {accel_reg, accel_val};
	HAL_I2C_Master_Transmit(&hi2c1, accel_address, cmd, 2, 50);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
}
void initMagnetometer(void){
	uint8_t magn_address = 0x3c;
	uint8_t magn_reg, magn_val;

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	magn_reg=0x00;
	magn_val=0x1c;
	uint8_t cmd[] = {magn_reg, magn_val};

	HAL_I2C_Master_Transmit(&hi2c1, magn_address, cmd, 2, 50);
	magn_reg=0x02;
	magn_val=0x00;
	cmd[0] = magn_reg;
	cmd[1] = magn_val;
	HAL_I2C_Master_Transmit(&hi2c1, magn_address, cmd, 2, 50);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

}

uint8_t readGyroscope(float* data){
	uint8_t address_out;
	uint8_t rxbuf[8];
	int16_t xgyro, ygyro, zgyro;

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	address_out = 0xc0 | 0x27;      // MSB high: read mode
	HAL_SPI_Transmit(&hspi1, &address_out, 1, 50);
	HAL_SPI_Receive(&hspi1, rxbuf, 8, 50);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	if (rxbuf[1] & 0x8) {
		xgyro = (rxbuf[3] << 8) | rxbuf[2];
		ygyro = (rxbuf[5] << 8) | rxbuf[4];
		zgyro = (rxbuf[7] << 8) | rxbuf[6];
		data[0] = (((float)xgyro) * mdps_per_digit)/1000.0;
		data[1] = (((float)ygyro) * mdps_per_digit)/1000.0;
		data[2] = (((float)zgyro) * mdps_per_digit)/1000.0;
		return 1;
	}
	return 0;
}
uint8_t readAccelerometer(float* data){
	//accelerometer and magnetometer lsm303dlhc
	//slave address = 00111xxb, whereas the xx bits are modified by the SDO/SA0 pin in order to modify the device address

	//magnetometer address 0x3c
	//0x67 might be for 100Hz but normally is 0x97 for 800Hz

	//HAL_I2C_MspInit(&hi2c1);

	uint8_t out[7];
	uint16_t xacc, yacc, zacc;
	uint8_t start_reg = 0x27 | 0x80;
	uint8_t accel_address = 0x32;

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_I2C_Master_Transmit(&hi2c1, accel_address, &start_reg, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, accel_address, out, 7, 50);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

	if (out[0] & 0x8) {
		xacc = (out[2] << 8) | out[1];
		yacc = (out[4] << 8) | out[3];
		zacc = (out[6] << 8) | out[5];
		// Accel scale is +- 2.0g
		data[0] = ((float)xacc)*(4.0/(65535.0))*9.81;
		data[1] = ((float)yacc)*(4.0/(65535.0))*9.81;
		data[2] = ((float)zacc)*(4.0/(65535.0))*9.81;
		return 1;
	}
	return 0;
}
uint8_t readMagnetometer(float* data){
	uint8_t magn_address = 0x3c;
	uint8_t start_reg = 0x03;
	uint8_t out[7];
	uint16_t xmagn, ymagn, zmagn;

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_I2C_Master_Transmit(&hi2c1, magn_address, &start_reg, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, magn_address, out, 7, 50);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

    xmagn = (out[0] << 8) | out[1];
    zmagn = (out[2] << 8) | out[3];
    ymagn = (out[4] << 8) | out[5];
    data[0] = ((float)xmagn)*1.22;
    data[1] = ((float)ymagn)*1.22;
    data[2] = ((float)zmagn)*1.22;
    return 1;
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00902025;	//0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;						//SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;    //SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
