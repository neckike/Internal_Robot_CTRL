/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
//#include "ultrasonic.h"
#include "usart.h"
#include "gpio.h"
//#include "internal_sensors.h"
//#include "can_stm.h"
#include "pdal.h"
#include "periphAL.h"
//#include "ControllerAreaNetwork.h"
#include "SensorDataHandler.h"

//From group E:
//GPIO_PIN_9	//LED3_RED		NORTH
//GPIO_PIN_8	//LED4_BLUE
//GPIO_PIN_10	//LED5_ORANGE
//GPIO_PIN_15	//LED6_GREEN	WEST
//GPIO_PIN_11	//LED7_GREEN	EAST
//GPIO_PIN_14	//LED8_ORANGE
//GPIO_PIN_12	//LED9_BLUE
//GPIO_PIN_13	//LED10_RED		SOUTH


//sensorPin right_sensor;
//sensorPin back_sensor;
//sensorPin left_sensor;
//sensorPin front_sensor;

//CanRxMsgTypeDef RxMessage;
//CanTxMsgTypeDef TxMessage;

ADC_HandleTypeDef hadc1;

//CAN_HandleTypeDef hcan;

//I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

//SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

//UART_HandleTypeDef huart4;
//UART_HandleTypeDef huart5;
//UART_HandleTypeDef huart2;
//UART_HandleTypeDef huart3;

DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

PCD_HandleTypeDef hpcd_USB_FS;

osThreadId defaultTaskHandle;

//static void floatToCAN(float data, can_frame_types_t *can_frame_types);
void SystemClock_Config(void);
static void MX_ADC1_Init(void);
//static void MX_CAN_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_USB_PCD_Init(void);
void StartDefaultTask(void const * argument);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

//static void canrx(void *pvParameters);
//static void cantx(void *pvParameters);
//static void gamTask(void *pvParameters);
//static void usb_test(void *pvParameters);
//static void ultrasonicTask(void *pvParameters);
static void mainTask(void *pvParameters);

//static void canSend(float data);
//static void canReceive(void);

int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();


  //in it commented, in msp commented, in usart, dma init commented.

  MX_ADC1_Init();
  MX_CAN_Init();		//can
  MX_I2C1_Init();		//accelerometer & magnetometer
  MX_I2C2_Init();
  MX_SPI1_Init();		//gyroscope
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();	//ultrasonic
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();


  initHardware();
  //ultrasonic_Init();

  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */

  //xTaskCreate(canrx, "rxTask", 100, NULL, 2, NULL);
  //xTaskCreate(cantx, "txTask", 100, NULL, 2, NULL);
  xTaskCreate(mainTask, "mainTask", 100, NULL, 2, NULL);
  //xTaskCreate(gamTask, "Gyroscope, Accelerometer, and Magnetometer Task", 100, NULL, 3, NULL);
  //xTaskCreate(ultrasonicTask, "Ultrasonic Task", 100, NULL, 3, NULL);
  //xTaskCreate(usb_test, "usbTask", 100, NULL, 3, NULL);

  //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  return 0;
  //while (1){}
  /* USER CODE END 3 */

}

//From group E:
//GPIO_PIN_9	//LED3_RED		NORTH
//GPIO_PIN_8	//LED4_BLUE
//GPIO_PIN_10	//LED5_ORANGE
//GPIO_PIN_15	//LED6_GREEN	WEST
//GPIO_PIN_11	//LED7_GREEN	EAST
//GPIO_PIN_14	//LED8_ORANGE
//GPIO_PIN_12	//LED9_BLUE
//GPIO_PIN_13	//LED10_RED		SOUTH
//static void floatToCAN(float data, can_frame_types_t *can_frame_types){
//		uint32_t aux = (uint32_t)(*(uint32_t*)&data);
//		uint8_t datal = (uint8_t)(aux &  0x000000FF);
//		uint8_t datah = (uint8_t)((aux & 0x0000FF00) >> 8);
//		can_frame_types->data[0] = datah;
//		can_frame_types->data[1] = datal;
//}
static void mainTask(void *pvParameters){

//	initGyroscope();
//	initAccelerometer();
//	initMagnetometer();
//
//	float gyro[3];
//	float accel[3];
//	float magn[3];

//	can_frame_types_t	sentMessage;
//	sentMessage.c_s_bit_id = SENSOR_DATA_BIT;
//	sentMessage.dlc = 1;
//	sentMessage.priority_id = PRIORITY_MEDIUM;
//
//	right_sensor.num_trig = GPIO_PIN_2;
//	right_sensor.grp_trig = (uint32_t) GPIOC;
//	right_sensor.num_echo = GPIO_PIN_3;
//	right_sensor.grp_echo = (uint32_t) GPIOC;
//
//	back_sensor.num_trig = GPIO_PIN_10;
//	back_sensor.grp_trig = (uint32_t) GPIOD;
//	back_sensor.num_echo = GPIO_PIN_11;
//	back_sensor.grp_echo = (uint32_t) GPIOD;
//
//	left_sensor.num_trig = GPIO_PIN_10;
//	left_sensor.grp_trig = (uint32_t) GPIOB;
//	left_sensor.num_echo = GPIO_PIN_11;
//	left_sensor.grp_echo = (uint32_t) GPIOB;
//
//	front_sensor.num_trig = GPIO_PIN_2;
//	front_sensor.grp_trig = (uint32_t) GPIOB;
//	front_sensor.num_echo = GPIO_PIN_7;
//	front_sensor.grp_echo = (uint32_t) GPIOE;

	while(1){
		//setMode(ONE_TIME_TRANSMISSION);
		//handleACommand();
//		//canReceive();
//		receivedMessage = can_receive_data_frame_pd();
//		//if(receive.ok == 1){
//			if(receivedMessage.c_s_bit_id ==SENSOR_DATA_BIT){	//receive.cs==1){	//sensor
//				//priority according to queues.
//
//				//if(receive.identity==0x1E || receive.identity==0x1F || receive.identity==0x20){	//gyroscope
//				if (receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_X ||
//						receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_Y ||
//						receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_Z){
//					readGyroscope(gyro);
//					if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_X){
//						//sentMessage.data[0] = gyro[0];
//						floatToCAN(gyro[0], &sentMessage);
//						can_transmit_data_frame_pd(&sentMessage);
//					}
//					else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_Y){
//						//sentMessage.data[0] = gyro[1];
//						floatToCAN(gyro[1], &sentMessage);
//						can_transmit_data_frame_pd(&sentMessage);
//					}
//					else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_Z){
//						//sentMessage.data[0] = gyro[2];
//						floatToCAN(gyro[2], &sentMessage);
//						can_transmit_data_frame_pd(&sentMessage);
//					}
//					/*switch(receivedMessage.topic_id){	//receive.identity){
//						case INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_X:
//							HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);		//BLUE
//							//canSend(gyro[0]);
//							sentMessage.data[0] = gyro[0];
//							can_transmit_data_frame_pd(&sentMessage);
//							//send.identity = gyro[0];
//						break;
//						case INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_Y:
//							canSend(gyro[1]);
//							//send.identity = gyro[1];
//						break;
//						case INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_Z:
//							canSend(gyro[2]);
//							//send.identity = gyro[2];
//						break;
//					}*/
//				}
//				else if (receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_X ||
//						receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_Y ||
//						receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_Z){
//					readAccelerometer(accel);
//					if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_X){
//						//sentMessage.data[0] = accel[0];
//						floatToCAN(accel[0], &sentMessage);
//						can_transmit_data_frame_pd(&sentMessage);
//					}
//					else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_Y){
//						//sentMessage.data[0] = accel[1];
//						floatToCAN(accel[1], &sentMessage);
//						can_transmit_data_frame_pd(&sentMessage);
//					}
//					else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_Z){
//						//sentMessage.data[0] = accel[2];
//						floatToCAN(accel[2], &sentMessage);
//						can_transmit_data_frame_pd(&sentMessage);
//					}
//				}
//				/*
//				else if(receive.identity==0x21 || receive.identity==0x22 || receive.identity==0x23){	//accelerometer
//					readAccelerometer(accel);
//					switch(receive.identity){
//						case 0x21:
//							canSend(accel[0]);
//							//send.identity = accel[0];
//						break;
//						case 0x22:
//							canSend(accel[1]);
//							//send.identity = accel[1];
//						break;
//						case 0x23:
//							canSend(accel[2]);
//							//send.identity = accel[2];
//						break;
//					}
//				}*/
//				else if (receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_X ||
//						receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_Y ||
//						receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_Z){
//					readMagnetometer(magn);
//					if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_X){
//						//sentMessage.data[0] = magn[0];
//						floatToCAN(magn[0], &sentMessage);
//						can_transmit_data_frame_pd(&sentMessage);
//					}
//					else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_Y){
//						//sentMessage.data[0] = magn[1];
//						floatToCAN(magn[1], &sentMessage);
//						can_transmit_data_frame_pd(&sentMessage);
//					}
//					else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_Z){
//						//sentMessage.data[0] = magn[2];
//						floatToCAN(magn[2], &sentMessage);
//						can_transmit_data_frame_pd(&sentMessage);
//					}
//				}
//				/*
//				else if(receive.identity==0x24 || receive.identity==0x25 || receive.identity==0x26){	//magnet
//					readMagnetometer(magn);
//					switch(receive.identity){
//						case 0x24:
//							canSend(magn[0]);
//							//send.identity = magn[0];
//						break;
//						case 0x25:
//							canSend(magn[1]);
//							//send.identity = magn[1];
//						break;
//						case 0x26:
//							canSend(magn[2]);
//							//send.identity = magn[2];
//						break;
//					}
//				}*/
//
//				else if (receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_1 ||
//						receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_2 ||
//						receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_3 ||
//						receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_4){
//
//					if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_1){
//						//sentMessage.data[0] = readUltrasonic(&right_sensor);
//						floatToCAN(readUltrasonic(&right_sensor), &sentMessage);
//						can_transmit_data_frame_pd(&sentMessage);
//					}
//					else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_2){
//						//sentMessage.data[0] = readUltrasonic(&back_sensor);
//						floatToCAN(readUltrasonic(&back_sensor), &sentMessage);
//						can_transmit_data_frame_pd(&sentMessage);
//					}
//					else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_3){
//						//sentMessage.data[0] = readUltrasonic(&left_sensor);
//						floatToCAN(readUltrasonic(&left_sensor), &sentMessage);
//						can_transmit_data_frame_pd(&sentMessage);
//					}
//					else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_4){
//						//sentMessage.data[0] = readUltrasonic(&front_sensor);
//						floatToCAN(readUltrasonic(&front_sensor), &sentMessage);
//						can_transmit_data_frame_pd(&sentMessage);
//					}
//				}
//				/*
//				else if(receive.identity==0x27 || receive.identity==0x28 || receive.identity==0x29 || receive.identity==0x2A){	//ultrasonic
//					switch(receive.identity){
//						case 0x27:
//							canSend(readUltrasonic(&right_sensor));
//							//send.identity = readUltrasonic(&right_sensor);
//						break;
//						case 0x28:
//							canSend(readUltrasonic(&back_sensor));
//							//send.identity = readUltrasonic(&back_sensor);
//						break;
//						case 0x29:
//							canSend(readUltrasonic(&left_sensor));
//							//send.identity = readUltrasonic(&left_sensor);
//						break;
//						case 0x2A:
//							canSend(readUltrasonic(&front_sensor));
//							//send.identity = readUltrasonic(&front_sensor);
//						break;
//					}
//				}*/
//			}
//			else if(receivedMessage.c_s_bit_id == COMMAND_DATA_BIT){	//receive.cs==0){	//command
//
//			}
		//}
//		else if(receive.ok == 0){
//			//ERROR
//		}

	vTaskDelay(100);
	}
}
/*
static void canReceive(){

	RxMessage.StdId = 0x0;
	RxMessage.ExtId= 0x0;
	RxMessage.RTR = CAN_RTR_DATA;
	RxMessage.IDE = CAN_ID_STD;
	RxMessage.DLC = 2;
	RxMessage.Data[0] = 0x00;
	RxMessage.Data[1] = 0x00;
	RxMessage.FMI = 0x0;
	RxMessage.FIFONumber=CAN_FIFO0;
	hcan.pRxMsg = &RxMessage;

	int errrx = HAL_CAN_Receive(&hcan, CAN_FIFO0,  500);

	if(errrx != HAL_OK){
		if(errrx == HAL_ERROR){
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);		//RED
		}
		else if(errrx==HAL_TIMEOUT){
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);			//RED	NORTH
		}
		receive.ok=0;
	}
	if(errrx == HAL_OK){
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);		//ORANGE
		receive.ok = 1;
		receive.cs = (0x04 & RxMessage.Data[0]) >> 2;
		receive.priority = 0x03 & RxMessage.Data[0];
		receive.identity = RxMessage.Data[1];
	}
}

static void canSend(float data){
	TxMessage.StdId = 0x11;
	TxMessage.ExtId= 0x01;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 2;

	uint32_t aux = (uint32_t)(*(uint32_t*)&data);

	//aux = (uint16_t)(data);
	uint8_t datal = (uint8_t)(aux &  0x000000FF);
	uint8_t datah = (uint8_t)((aux & 0x0000FF00) >> 8);

	TxMessage.Data[0] = datah;
	TxMessage.Data[1] = datal;
	hcan.pTxMsg = &TxMessage;

	int errtx=HAL_CAN_Transmit(&hcan,  500);

	if(errtx == HAL_OK){
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);		//GREEN
	}
}*/

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM8|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}



// USB init function
static void MX_USB_PCD_Init(void)
{

  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.ep0_mps = DEP0CTL_MPS_64;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

//static void canrx(void *pvParameters){
//	/*CAN  Identifier
//	 * 11bits ->
//	 *
//	 *|	0	|	1		2	|	3		4		5		6		7		8		9		10	|
//	 *
//	 *|	C/S	|	Priority	|							Identity							|
//	 *
//	 *|	1	|		2		|								8								|
//	 *
//	 * C/S
//	 * 0 = Command
//	 * 1 = Sensor
//	 *
//	 * Priority
//	 * 00 = Urgent
//	 * 01 = High
//	 * 10 = Medium
//	 * 11 = Low
//	 *
//	 *
//	 * Identity:
//	 *
//		1E		Gyroscope x
//		1F		Gyroscope y
//		20		Gyroscope z
//		21		Accelerometer x
//		22		Accelerometer y
//		23		Accelerometer z
//		24		Magnetometer x
//		25		Magnetometer y
//		26		Magnetometer z
//		27		Ultrasonic sensor 1
//		28		Ultrasonic sensor 2
//		29		Ultrasonic sensor 3
//		2A		Ultrasonic sensor 4
//		2B		Velocity x	Robot Velocity
//		2C		Velocity y
//		2D		Velocity z
//	 *
//	 *
//	 *
//	 */
//
//	//CanRxMsgTypeDef RxMessage;
//
////	RxMessage.StdId = 0x0;
////	RxMessage.ExtId= 0x0;
////	RxMessage.RTR = CAN_RTR_DATA;
////	RxMessage.IDE = CAN_ID_STD;
////	RxMessage.DLC = 2;
////	RxMessage.Data[0] = 0x00;
////	RxMessage.Data[1] = 0x00;
////	RxMessage.FMI = 0x0;
////	RxMessage.FIFONumber=CAN_FIFO0;
////	hcan.pRxMsg = &RxMessage;
//
//	while(1){
//		int errrx=HAL_CAN_Receive(&hcan, CAN_FIFO0,  500);
//		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
//		if(errrx != HAL_OK){
//			if(errrx == HAL_ERROR){
//				//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);		//RED
//			}
//			else if(errrx==HAL_TIMEOUT){
//				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);			//RED	NORTH
//			}
//		}
//		if(errrx == HAL_OK){
//			receive.cs = 0x04 & RxMessage.Data[0];
//			receive.priority = 0x03 & RxMessage.Data[0];
//			receive.identity = RxMessage.Data[1];
//
////			if(RxMessage.Data[0] == 0xDA){
////				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);		//ORANGE
////			}
//
//		}
//		vTaskDelay(2000);
//	}
//}
//static void cantx(void *pvParameters){
//
////	TxMessage.StdId = 0x11;
////	TxMessage.ExtId= 0x01;
////	TxMessage.RTR = CAN_RTR_DATA;
////	TxMessage.IDE = CAN_ID_STD;
////	TxMessage.DLC = 2;
////	TxMessage.Data[0] = 0xAA;
////	TxMessage.Data[1] = 0x99;
////	hcan.pTxMsg = &TxMessage;
//
//	while(1) {
//		int errtx=HAL_CAN_Transmit(&hcan,  500);
////		if(errtx != HAL_OK){
////			if(errtx == HAL_ERROR){
////				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);		//RED
////			}
////			else if(errtx==HAL_TIMEOUT){
////				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);		//BLUE
////			}
////		}
////		else{
////			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);		//GREEN
////		}
//		vTaskDelay(2000);
//	}
//}
//
//static void ultrasonicTask(void *pvParameters){
//	float distance;
//	int flag=0;
//	//initUltrasonic();
//
//	right_sensor.num_trig = GPIO_PIN_2;
//	right_sensor.grp_trig = (uint32_t) GPIOC;
//	right_sensor.num_echo = GPIO_PIN_3;
//	right_sensor.grp_echo = (uint32_t) GPIOC;
//
//	back_sensor.num_trig = GPIO_PIN_10;
//	back_sensor.grp_trig = (uint32_t) GPIOD;
//	back_sensor.num_echo = GPIO_PIN_11;
//	back_sensor.grp_echo = (uint32_t) GPIOD;
//
//	left_sensor.num_trig = GPIO_PIN_10;
//	left_sensor.grp_trig = (uint32_t) GPIOB;
//	left_sensor.num_echo = GPIO_PIN_11;
//	left_sensor.grp_echo = (uint32_t) GPIOB;
//
//	front_sensor.num_trig = GPIO_PIN_2;
//	front_sensor.grp_trig = (uint32_t) GPIOB;
//	front_sensor.num_echo = GPIO_PIN_7;
//	front_sensor.grp_echo = (uint32_t) GPIOE;
//
//	while(1){
//		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);			//LED4_BLUE
//
//		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){	//button bounce elimination
//			vTaskDelay(100);
//			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
//				flag = (flag + 1) % 5;
//			}
//		}
//		switch(flag){
//		case 1:
//			distance = readUltrasonic(&right_sensor);
//			if (distance <0) {
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);			//LED10_RED		SOUTH
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);			//LED9_BLUE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);			//LED5_ORANGE
//			}
//			else if (distance > 10) {
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);			//LED9_BLUE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);			//LED5_ORANGE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);			//LED10_RED		SOUTH
//			}
//			else {
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);			//LED9_BLUE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);			//LED5_ORANGE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);			//LED10_RED		SOUTH
//			}
//		break;
//
//		case 2:
//			distance = readUltrasonic(&back_sensor);
//			if (distance <0) {
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);			//LED10_RED		SOUTH
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);			//LED9_BLUE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);			//LED5_ORANGE
//			}
//			else if (distance > 10) {
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);			//LED9_BLUE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);			//LED5_ORANGE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);			//LED10_RED		SOUTH
//			}
//			else {
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);			//LED9_BLUE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);			//LED5_ORANGE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);			//LED10_RED		SOUTH
//			}
//		break;
//
//		case 3:
//			distance = readUltrasonic(&left_sensor);
//			if (distance <0) {
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);			//LED10_RED		SOUTH
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);			//LED9_BLUE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);			//LED5_ORANGE
//			}
//			else if (distance > 10) {
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);			//LED9_BLUE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);			//LED5_ORANGE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);			//LED10_RED		SOUTH
//			}
//			else {
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);			//LED9_BLUE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);			//LED5_ORANGE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);			//LED10_RED		SOUTH
//			}
//		break;
//
//		case 4:
//			distance = readUltrasonic(&front_sensor);
//			if (distance <0) {
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);			//LED10_RED		SOUTH
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);			//LED9_BLUE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);			//LED5_ORANGE
//			}
//			else if (distance > 10) {
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);			//LED9_BLUE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);			//LED5_ORANGE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);			//LED10_RED		SOUTH
//			}
//			else {
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);			//LED9_BLUE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);			//LED5_ORANGE
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);			//LED10_RED		SOUTH
//			}
//		break;
//
//		default:
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);			//LED9_BLUE
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);			//LED5_ORANGE
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);			//LED10_RED		SOUTH
//		break;
//
//		}
//		// Give some time to sensor
//		vTaskDelay(100);
//
//	}
//}
//
//static void usb_test(void *pvParameters){
//	uint8_t address = 0x00;
//	uint32_t len = 0xf;
//	uint8_t tbuff=0x01;
//	uint8_t pdev = 0x0;
//	while(1){
//		//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);	//LED4_BLUE
//		HAL_PCD_MspInit(&hpcd_USB_FS);
//		__HAL_RCC_USB_CLK_ENABLE();
//		hpcd_USB_FS.pData = &pdev;
//		HAL_PCD_Start(&hpcd_USB_FS);
//		HAL_PCD_DevConnect(&hpcd_USB_FS);
//		HAL_PCD_SetAddress(&hpcd_USB_FS, address);
//		HAL_PCD_EP_Open(&hpcd_USB_FS, PCD_ENDP0, DEP0CTL_MPS_64, PCD_EP_TYPE_BULK);
//
//		if(HAL_PCD_EP_Transmit(&hpcd_USB_FS, PCD_ENDP0, &tbuff, len) == HAL_OK){
//			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);	//LED7
//		}
//		vTaskDelay(1000);
//	}
//}
//static void gamTask(void *pvParameters){
//	int flag=0;
//	initGyroscope();
//	initAccelerometer();
//	initMagnetometer();
//
//	float gyro[3];
//	float accel[3];
//	float magn[3];
//
//	while(1){
//
//		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){	//button bounce elimination
//			vTaskDelay(100);
//			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
//				flag = (flag + 1) % 4;
//			}
//		}
//
//		switch(flag){
//		case 1:
//			if(readGyroscope(gyro)){
//				//Testing purposes only.
//				if(gyro[0]>100){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);	//LED10_RED		SOUTH
//				}
//				else if(gyro[0]<-270){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//				}
//				else if(gyro[1]>150){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//				}
//				else if(gyro[1]<-230){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//				}
//				else if(gyro[2]>150){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);	//LED4_BLUE
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);	//LED5_ORANGE
//				}
//				else if(gyro[2]<-150){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);	//LED4_BLUE
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);	//LED5_ORANGE
//				}
//				else{
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);	//LED4_BLUE
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);	//LED5_ORANGE
//				}//End of testing purposes only.
//			}
//		break;
//
//		case 2:
//			if(readAccelerometer(accel)){
//				if(accel[0]>20){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);	//LED10_RED		SOUTH
//				}
//				else if(accel[0]<-20){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//				}
//				else if(accel[1]>20){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//				}
//				else if(accel[1]<-20){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//				}
//				else if(accel[2]>10){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);	//LED4_BLUE
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);	//LED5_ORANGE
//				}
//				else if(accel[2]<-10){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);	//LED4_BLUE
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);	//LED5_ORANGE
//				}
//				else{
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);	//LED4_BLUE
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);	//LED5_ORANGE
//				}//End of testing purposes only.
//			}
//		break;
//		case 3:
//			if(readMagnetometer(magn)){
//				if(magn[0]>60000){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);	//LED10_RED		SOUTH
//				}
//				else if(magn[0]<-60000){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//				}
//				else if(magn[1]>60000){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//				}
//				else if(magn[1]<-60000){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//				}
//				else if(magn[2]>60000){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);	//LED4_BLUE
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);	//LED5_ORANGE
//				}
//				else if(magn[2]<-60000){
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);	//LED4_BLUE
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);	//LED5_ORANGE
//				}
//				else{
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);	//LED4_BLUE
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);	//LED5_ORANGE
//				}//End of testing purposes only.
//			}
//		break;
//		default:
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	//LED3_RED		NORTH
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	//LED6_GREEN	WEST
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	//LED7_GREEN	EAST
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);	//LED10_RED		SOUTH
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);	//LED4_BLUE
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);	//LED5_ORANGE
//		break;
//		}
//		vTaskDelay(500);
//	}
//}

