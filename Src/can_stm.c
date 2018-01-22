/*
 * can_stm.c
 *
 *  Created on: Jan 5, 2018
 *      Author: hector
 */
#include "cmsis_os.h"
#include "gpio.h"
#include "can_stm.h"

/*CAN_HandleTypeDef hcan;
CanRxMsgTypeDef RxMessage;
CanTxMsgTypeDef TxMessage;

canFrame receive;

void canReceive(){

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

void canSend(float data){
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

/* CAN init function */
//void MX_CAN_Init(void){
//
//// 24MHz/[(SJW+BS1+BS2)*Prescaler] = [freq] right now is 500k
//
//  CAN_FilterConfTypeDef  sFilterConfig1;
//
//  hcan.Instance = CAN;
//
//  hcan.Init.Mode = CAN_MODE_NORMAL;
//  hcan.Init.SJW = CAN_SJW_1TQ;
//  hcan.Init.BS1 = CAN_BS1_2TQ;		//before it was CAN_BS1_11TQ
//  hcan.Init.BS2 = CAN_BS2_3TQ;		//before it was CAN_BS2_3TQ
//  hcan.Init.TTCM = DISABLE;
//  hcan.Init.ABOM = DISABLE;
//  hcan.Init.AWUM = DISABLE;
//  hcan.Init.NART = DISABLE;
//  hcan.Init.RFLM = DISABLE;
//  hcan.Init.TXFP = DISABLE;
//
//  hcan.Init.Prescaler = 8;					 	//32
//
//  if (HAL_CAN_Init(&hcan) != HAL_OK){
//    _Error_Handler(__FILE__, __LINE__);
//  }
//
//	sFilterConfig1.FilterNumber = 1;
//	sFilterConfig1.FilterMode = CAN_FILTERMODE_IDMASK;
//	sFilterConfig1.FilterScale = CAN_FILTERSCALE_32BIT;
//	sFilterConfig1.FilterIdHigh = 0x0000;
//	sFilterConfig1.FilterIdLow = 0x0000;
//	sFilterConfig1.FilterMaskIdHigh = 0x0000;
//	sFilterConfig1.FilterMaskIdLow = 0x0000;
//	sFilterConfig1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//	sFilterConfig1.FilterActivation = ENABLE;
//	sFilterConfig1.BankNumber = 14;
//
//
//   if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig1) != HAL_OK){
//	   _Error_Handler(__FILE__, __LINE__);
//   }
//
//}
