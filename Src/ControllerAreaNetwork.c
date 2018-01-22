/*
 * ControllerAreaNetwork.c
 *
 *  Created on: May 4, 2017
 *      Author: uwlau
 */
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "ControllerAreaNetwork.h"
#include <unistd.h>


#ifdef rPI_DAEbot
	#include <linux/can.h>
	#include <linux/can/raw.h>
	#include <net/if.h>
	#include <sys/types.h>
	#include <sys/socket.h>
	#include <sys/ioctl.h>

#endif

#ifdef STM32F3_DAEbot
	#include "main.h"
	#include "stm32f3xx_hal.h"
	#include "cmsis_os.h"
	#include "ultrasonic.h"
	#include "usart.h"
	#include "gpio.h"
	#include "internal_sensors.h"
	#include "can_stm.h"


	CAN_HandleTypeDef hcan;
	CanRxMsgTypeDef RxMessage;
	CanTxMsgTypeDef TxMessage;
	can_frame_types_t receivedMessage;
	int errrx;
	int errtx;
#endif

#ifdef rPI_DAEbot
inline ExitStatus_t can_bind_socket_pd (can_socket_t *can_socket)
{

	if((can_socket->socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		return STATUS_ERROR;
	}

	strcpy(can_socket->ifr.ifr_ifrn.ifrn_name, "can0");

	ioctl(can_socket->socket, SIOCGIFINDEX, &can_socket->ifr);

	can_socket->addr.can_family = AF_CAN;

	can_socket->addr.can_ifindex = can_socket->ifr.ifr_ifru.ifru_ivalue;

	if(bind(can_socket->socket, (struct sockaddr *)&can_socket->addr, sizeof(can_socket->addr)) < 0) {
		return STATUS_ERROR;
	}


	return STATUS_OK;

}
#endif

#ifdef rPI_DAEbot
inline ExitStatus_t can_transmit_data_frame_pd(can_frame_types_t *can_frame_types, can_socket_t *can_socket) {

		struct can_frame frame;
	//#endif

	//#ifdef rPI_DAEbot
		// code the identifier
		frame.can_id = can_code_identifier_pd(&can_frame_types->priority_id, &can_frame_types->c_s_bit_id, &can_frame_types->topic_id);
		// data lenght (1 = 8 Bit)
		frame.can_dlc = can_frame_types->dlc;
		// as dlc is one, we only have data[0] (8 Bit)
		for (uint8_t i = 0; (i + 1) <= frame.can_dlc; i++)
		{
			frame.data[i] = can_frame_types->data[i];
		}
	//#endif

	//#ifdef rPI_DAEbot
		write(can_socket->socket, &frame, sizeof(frame));
		return STATUS_OK;
	}
#endif

	//#ifdef ARDUINO2560_DAEbot
	//#endif
#ifdef STM32F3_DAEbot
	inline ExitStatus_t can_transmit_data_frame_pd(can_frame_types_t *can_frame_types) {
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.IDE =  CAN_ID_EXT;
		TxMessage.ExtId= can_code_identifier_pd(&can_frame_types->priority_id, &can_frame_types->c_s_bit_id, &can_frame_types->topic_id);
		TxMessage.DLC = can_frame_types->dlc;

//		uint32_t aux = (uint32_t)(*(uint32_t*)&data);	//data is float
//
//		//aux = (uint16_t)(data);
//		uint8_t datal = (uint8_t)(aux &  0x000000FF);
//		uint8_t datah = (uint8_t)((aux & 0x0000FF00) >> 8);
//
//		TxMessage.Data[0] = datah;
//		TxMessage.Data[1] = datal;

		// as dlc is one, we only have data[0] (8 Bit)
		for (uint8_t i = 0; (i + 1) <= TxMessage.DLC; i++)
		{
			TxMessage.Data[i] = can_frame_types->data[i];
		}
		hcan.pTxMsg = &TxMessage;

		errtx=HAL_CAN_Transmit(&hcan,  500);

		if(errtx == HAL_OK){
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);		//GREEN
		}
	//#endif
	//#ifdef STM32F3_DAEbot
		//HAL_CAN_Transmit(&hcan,  500);
		return STATUS_OK;
	}
#endif


#ifdef rPI_DAEbot
	inline can_frame_types_t can_receive_data_frame_pd (can_socket_t *can_socket) {

		can_frame_types_t can_frame_types;


			read(can_socket->socket, &frame, sizeof(frame));

			//can_frame_types = can_decode_identifier_pd(&frame.can_id);
			can_frame_types.dlc = frame.can_dlc;

			for (uint8_t i = 0; (i + 1) <= frame.can_dlc ;i++)
			{
				can_frame_types.data[i] = frame.data[i];
			}

			return can_frame_types;


		//#ifdef ARDUINO2560_DAEbot
		//#endif

	}
#endif

#ifdef STM32F3_DAEbot
	inline can_frame_types_t can_receive_data_frame_pd (void) {

		can_frame_types_t can_frame_types;
		//hcan.pRxMsg = &RxMessage;
		errrx = HAL_CAN_Receive(&hcan, CAN_FIFO0,  500);
		if(errrx != HAL_OK){
			if(errrx == HAL_ERROR){
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);		//RED
			}
			else if(errrx==HAL_TIMEOUT){
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);			//RED	NORTH
			}
			//receive.ok=0;
		}
		//can_frame_types = can_decode_identifier_pd(&frame.can_id);
		can_frame_types.dlc = hcan.pRxMsg->DLC;

		for (uint8_t i = 0; (i + 1) <= hcan.pRxMsg->DLC ;i++)
		{
			can_frame_types.data[i] = hcan.pRxMsg->Data[i];
		}

		return can_frame_types;
	}
#endif


inline uint16_t can_code_identifier_pd(priority_can_id_t *priority_can_id, c_s_bit_can_id_t *c_s_bit_can_id, topic_can_id_t *topic_can_id) {

	// combines c_s_bit and priority bit
	uint8_t highest_bit = *priority_can_id + (*c_s_bit_can_id * 4);

	// combines "hightes bit" and topic ID and returns it
	return ((highest_bit * 0x100) + *topic_can_id);
}
#ifdef rPI_DAEbot
	inline can_frame_types_t can_decode_identifier_pd(canid_t *canid) {

		can_frame_types_t can_frame_types;

		// Get the hightes bit by bit-shifting
		uint8_t highest_bit = *canid >> 8;

		if (highest_bit < 0x4)
		{
			can_frame_types.c_s_bit_id = COMMAND_DATA_BIT;
			can_frame_types.priority_id = (priority_can_id_t)highest_bit;
			can_frame_types.topic_id = (topic_can_id_t) (*canid - (highest_bit * 0x100));
		}
		else
		{
			can_frame_types.c_s_bit_id = SENSOR_DATA_BIT;
			can_frame_types.priority_id = (priority_can_id_t)(highest_bit - 0x4);
			can_frame_types.topic_id = (topic_can_id_t) (*canid - (highest_bit * 0x100));
		}

		return can_frame_types;

	}
#endif

#ifdef STM32F3_DAEbot
	void MX_CAN_Init(void){

	// 24MHz/[(SJW+BS1+BS2)*Prescaler] = [freq] right now is 500k

	  CAN_FilterConfTypeDef  sFilterConfig1;

	  hcan.Instance = CAN;

	  hcan.Init.Mode = CAN_MODE_NORMAL;
	  hcan.Init.SJW = CAN_SJW_1TQ;
	  hcan.Init.BS1 = CAN_BS1_2TQ;		//before it was CAN_BS1_11TQ
	  hcan.Init.BS2 = CAN_BS2_3TQ;		//before it was CAN_BS2_3TQ
	  hcan.Init.TTCM = DISABLE;
	  hcan.Init.ABOM = DISABLE;
	  hcan.Init.AWUM = DISABLE;
	  hcan.Init.NART = DISABLE;
	  hcan.Init.RFLM = DISABLE;
	  hcan.Init.TXFP = DISABLE;

	  hcan.Init.Prescaler = 8;					 	//32

	  if (HAL_CAN_Init(&hcan) != HAL_OK){
	    _Error_Handler(__FILE__, __LINE__);
	  }

		sFilterConfig1.FilterNumber = 1;
		sFilterConfig1.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig1.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig1.FilterIdHigh = 0x0000;
		sFilterConfig1.FilterIdLow = 0x0000;
		sFilterConfig1.FilterMaskIdHigh = 0x0000;
		sFilterConfig1.FilterMaskIdLow = 0x0000;
		sFilterConfig1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
		sFilterConfig1.FilterActivation = ENABLE;
		sFilterConfig1.BankNumber = 14;


	   if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig1) != HAL_OK){
		   _Error_Handler(__FILE__, __LINE__);
	   }

	}
#endif
