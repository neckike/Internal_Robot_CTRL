#ifndef __can_stm_H
#define __can_stm_H
#ifdef __cplusplus
extern "C"
{
#endif
#include "stm32f3xx_hal.h"
#include "main.h"

/*extern CAN_HandleTypeDef hcan;
extern CanRxMsgTypeDef RxMessage;
extern CanTxMsgTypeDef TxMessage;*/

typedef struct canFrame{
	uint8_t	cs;
	uint8_t priority;
	uint8_t identity;
	uint8_t ok;
}canFrame;


//typedef __uint8_t canid_t;
//
//struct can_frame_stm {
//	canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
//	__uint8_t    can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
//	__uint8_t    __pad;   /* padding */
//	__uint8_t    __res0;  /* reserved / padding */
//	__uint8_t    __res1;  /* reserved / padding */
//	__uint8_t    data[8] __attribute__((aligned(8)));
//};
//


/*
extern canFrame receive;

void canSend(float data);
void canReceive(void);
void MX_CAN_Init(void);*/

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */
