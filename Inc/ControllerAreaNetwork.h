#ifndef _CONTROLLERAREANETWORK_H_
#define _CONTROLLERAREANETWORK_H_

#include "periphAL.h"
#include <stdint.h>
//#include <linux/can.h>
#ifdef STM32F3_DAEbot
	#include "stm32f3xx_hal.h"
	#include "main.h"

	extern CAN_HandleTypeDef hcan_test;
	extern CanRxMsgTypeDef RxMessage_test;
	extern CanTxMsgTypeDef TxMessage_test;

	extern int errrx;
	extern int errtx;

#endif
/**
 * @brief Priority for CAN identifier.
 */
typedef enum {
	PRIORITY_URGENT	= 0x0u,
	PRIORITY_HIGH	= 0x1u,
	PRIORITY_MEDIUM	= 0x2u,
	PRIORITY_LOW	= 0x3u,
} priority_can_id_t;

/**
 * @brief C/S Bit for CAN identifier.
 */
typedef enum {
	COMMAND_DATA_BIT = 0x0u,
	SENSOR_DATA_BIT	 = 0x1u,
} c_s_bit_can_id_t;

/**
 * @brief CAN Topic ID's.
 */
typedef enum topic_id_can {
	HEALTH_CTRL_ID_CURRENT_VOLTAGE 				= 0x00u,
	HEALTH_CTRL_ID_CURRENT_POWER 				= 0x01u,
	HEALTH_CTRL_ID_REMAINING_CAPACITY 			= 0x02u,
	HEALTH_CTRL_ID_BATTERY_TEMPERATURE			= 0x03u,
	HEALTH_CTRL_ID_ENVIRONMENT_TEMPERATURE		= 0x04u,
	HEALTH_CTRL_ID_ENVIRONMENT_AIR_PRESSURE		= 0x05u,
	HEALTH_CTRL_ID_ENVIRONMENT_HUMIDITY			= 0x06u,
	HEALTH_CTRL_ID_MAGNETIC_FIELD				= 0x07u,
	HEALTH_CTRL_ID_MOTOR_1_TEMPERATURE			= 0x08u,
	HEALTH_CTRL_ID_MOTOR_2_TEMPERATURE			= 0x09u,
	HEALTH_CTRL_ID_MOTOR_3_TEMPERATURE			= 0x0Au,
	HEALTH_CTRL_ID_MOTOR_4_TEMPERATURE			= 0x0Bu,
	HEALTH_CTRL_ID_MOTOR_DRIVER_1_TEMPERATURE	= 0x0Cu,
	HEALTH_CTRL_ID_MOTOR_DRIVER_2_TEMPERATURE	= 0x0Du,
	HEALTH_CTRL_ID_RELAY_1						= 0x0Eu,
	HEALTH_CTRL_ID_RELAY_2						= 0x0Fu,
	HEALTH_CTRL_ID_RELAY_3						= 0x10u,
	HEALTH_CTRL_ID_RELAY_4						= 0x11u,
	HEALTH_CTRL_ID_RELAY_5						= 0x12u,
	HEALTH_CTRL_ID_RELAY_6						= 0x13u,
	HEALTH_CTRL_ID_RELAY_7						= 0x14u,
	HEALTH_CTRL_ID_RELAY_8						= 0x15u,
	HEALTH_CTRL_ID_RELAY_9						= 0x16u,
	HEALTH_CTRL_ID_RELAY_10						= 0x17u,
	HEALTH_CTRL_ID_COMP_STATUS					= 0x1Bu,
	HEALTH_CTRL_ID_PULSEAT 						= 0x1Cu,
	HEALTH_CTRL_ID_PULSEAT_B					= 0x1Du,
	INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_X			= 0x1Eu,
	INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_Y			= 0x1Fu,
	INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_Z			= 0x20u,
	INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_X		= 0x21u,
	INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_Y		= 0x22u,
	INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_Z		= 0x23u,
	INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_X		= 0x24u,
	INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_Y		= 0x25u,
	INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_Z		= 0x26u,
	INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_1	= 0x27u,
	INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_2	= 0x28u,
	INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_3	= 0x29u,
	INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_4	= 0x2Au,
	INTERNAL_ROBOT_CTRL_ID_VELOCITY_X			= 0x2Bu,
	INTERNAL_ROBOT_CTRL_ID_VELOCITY_Y			= 0x2Cu,
	INTERNAL_ROBOT_CTRL_ID_VELOCITY_Z			= 0x2Du,
	INTERNAL_ROBOT_CTRL_ID_COMP_STATUS			= 0x39u,
	INTERNAL_ROBOT_CTRL_ID_PULSEAT  			= 0x3Au,
	INTERNAL_ROBOT_CTRL_ID_PULSEAT_B			= 0x3Bu,
	CAM_CTRL_ID_COMP_STATUS						= 0x57u,
	CAM_CTRL_ID_PULSEAT							= 0x58u,
	CAM_CTRL_ID_PULSEAT_B						= 0x59u,
	RPI_OPERATOR_ID_COMP_STATUS					= 0x75u,
	RPI_OPERATOR_ID_PULSEAT						= 0x76u,
	RPI_OPERATOR_ID_PULSEAT_B					= 0x77u,
	OPERATOR_PLUS_ID_COMP_STATUS				= 0x93u,
	OPERATOR_PLUS_ID_PULSEAT					= 0x94u,
	OPERATOR_PLUS_ID_PULSEAT_B					= 0x95u,
	M2M_GW_ID_COMP_STATUS						= 0xB1u,
	M2M_GW_ID_PULSEAT							= 0xB2u,
	M2M_GW_ID_PULSEAT_B							= 0xB3u
} topic_can_id_t;

/**
 * @brief CAN Socket.
 */
typedef struct can_socket {
	int socket;
#ifdef rPI_DAEbot
	struct sockaddr_can addr;
	struct ifreq ifr;
#endif
} can_socket_t;

/**
 * @brief CAN Frame Types.
 */
typedef struct can_frame_types {
	priority_can_id_t priority_id;
	c_s_bit_can_id_t c_s_bit_id;
	topic_can_id_t topic_id;
	uint8_t dlc;
	uint8_t data[15];
} can_frame_types_t;

#ifdef __cplusplus
extern "C" {
#endif

uint16_t can_code_identifier_pd(priority_can_id_t *priority_can_id, c_s_bit_can_id_t *c_s_bit_can_id, topic_can_id_t *topic_can_id);

#ifdef rPI_DAEbot
	ExitStatus_t can_bind_socket_pd(can_socket_t *can_socket);
	can_frame_types_t can_decode_identifier_pd(canid_t *canid);
	ExitStatus_t can_transmit_data_frame_pd(can_frame_types_t *can_frame_types, can_socket_t *can_socket);
	can_frame_types_t can_receive_data_frame_pd(can_socket_t *can_socket);
#endif

#ifdef STM32F3_DAEbot
	extern can_frame_types_t receivedMessage;
	void MX_CAN_Init(void);
	ExitStatus_t can_transmit_data_frame_pd(can_frame_types_t *can_frame_types);
	can_frame_types_t can_receive_data_frame_pd(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _CONTROLLERAREANETWORK_H_ */
