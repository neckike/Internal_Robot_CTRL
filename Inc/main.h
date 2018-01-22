/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define DRDY_Pin GPIO_PIN_2
#define DRDY_GPIO_Port GPIOE
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define MEMS_INT3_Pin GPIO_PIN_4
#define MEMS_INT3_GPIO_Port GPIOE
#define MEMS_INT4_Pin GPIO_PIN_5
#define MEMS_INT4_GPIO_Port GPIOE
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define SPI2_KLICK_SCK_Pin GPIO_PIN_9
#define SPI2_KLICK_SCK_GPIO_Port GPIOF
#define PWM4_Pin GPIO_PIN_10
#define PWM4_GPIO_Port GPIOF
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOF
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOF
#define M4_A_Pin GPIO_PIN_0
#define M4_A_GPIO_Port GPIOC
#define M4_B_Pin GPIO_PIN_1
#define M4_B_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define RS485_DE_Pin GPIO_PIN_1
#define RS485_DE_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_2
#define RS485_TX_GPIO_Port GPIOA
#define RS485_RX_Pin GPIO_PIN_3
#define RS485_RX_GPIO_Port GPIOA
#define CLICK_ADC_Pin GPIO_PIN_4
#define CLICK_ADC_GPIO_Port GPIOF
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MISOA7_Pin GPIO_PIN_7
#define SPI1_MISOA7_GPIO_Port GPIOA
#define CLICK_INT_Pin GPIO_PIN_0
#define CLICK_INT_GPIO_Port GPIOB
#define CLICK_RST_Pin GPIO_PIN_1
#define CLICK_RST_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_9
#define LD3_GPIO_Port GPIOE
#define LD5_Pin GPIO_PIN_10
#define LD5_GPIO_Port GPIOE
#define LD7_Pin GPIO_PIN_11
#define LD7_GPIO_Port GPIOE
#define LD9_Pin GPIO_PIN_12
#define LD9_GPIO_Port GPIOE
#define LD10_Pin GPIO_PIN_13
#define LD10_GPIO_Port GPIOE
#define LD8_Pin GPIO_PIN_14
#define LD8_GPIO_Port GPIOE
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOE
#define SPI2_KLICK_CS_Pin GPIO_PIN_13
#define SPI2_KLICK_CS_GPIO_Port GPIOB
#define SPI2_KLICK_MISO_Pin GPIO_PIN_14
#define SPI2_KLICK_MISO_GPIO_Port GPIOB
#define SPI2_KLICK_MOSI_Pin GPIO_PIN_15
#define SPI2_KLICK_MOSI_GPIO_Port GPIOB
#define CLICK_UART_TX_Pin GPIO_PIN_8
#define CLICK_UART_TX_GPIO_Port GPIOD
#define CLICK_UART_RX_Pin GPIO_PIN_9
#define CLICK_UART_RX_GPIO_Port GPIOD
#define ENC4_A_Pin GPIO_PIN_12
#define ENC4_A_GPIO_Port GPIOD
#define ENC4_B_Pin GPIO_PIN_13
#define ENC4_B_GPIO_Port GPIOD
#define M1_B_Pin GPIO_PIN_14
#define M1_B_GPIO_Port GPIOD
#define M1_A_Pin GPIO_PIN_15
#define M1_A_GPIO_Port GPIOD
#define ENC3_A_Pin GPIO_PIN_6
#define ENC3_A_GPIO_Port GPIOC
#define ENC3_B_Pin GPIO_PIN_7
#define ENC3_B_GPIO_Port GPIOC
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOC
#define ENC1_A_Pin GPIO_PIN_8
#define ENC1_A_GPIO_Port GPIOA
#define ENC1_B_Pin GPIO_PIN_9
#define ENC1_B_GPIO_Port GPIOA
#define I2C2_CLICK_SDA_Pin GPIO_PIN_10
#define I2C2_CLICK_SDA_GPIO_Port GPIOA
#define DM_Pin GPIO_PIN_11
#define DM_GPIO_Port GPIOA
#define DP_Pin GPIO_PIN_12
#define DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define I2C2_CLICK_SCL_Pin GPIO_PIN_6
#define I2C2_CLICK_SCL_GPIO_Port GPIOF
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define PI_UART_TX_Pin GPIO_PIN_10
#define PI_UART_TX_GPIO_Port GPIOC
#define PI_UART_RX_Pin GPIO_PIN_11
#define PI_UART_RX_GPIO_Port GPIOC
#define USB_UART_TX_Pin GPIO_PIN_12
#define USB_UART_TX_GPIO_Port GPIOC
#define USB_UART_RX_Pin GPIO_PIN_2
#define USB_UART_RX_GPIO_Port GPIOD
#define ENC2_A_Pin GPIO_PIN_3
#define ENC2_A_GPIO_Port GPIOD
#define ENC2_B_Pin GPIO_PIN_4
#define ENC2_B_GPIO_Port GPIOD
#define M2_B_Pin GPIO_PIN_6
#define M2_B_GPIO_Port GPIOD
#define M2_A_Pin GPIO_PIN_7
#define M2_A_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_4
#define PWM2_GPIO_Port GPIOB
#define PWM3_Pin GPIO_PIN_5
#define PWM3_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define M3_B_Pin GPIO_PIN_8
#define M3_B_GPIO_Port GPIOB
#define M3_A_Pin GPIO_PIN_9
#define M3_A_GPIO_Port GPIOB
#define MEMS_INT1_Pin GPIO_PIN_0
#define MEMS_INT1_GPIO_Port GPIOE
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
