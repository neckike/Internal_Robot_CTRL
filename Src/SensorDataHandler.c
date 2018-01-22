/*
 * SensorDataHandler.c
 *
 *  Created on: Jan 19, 2018
 *      Author: hector
 */
#include "cmsis_os.h"
#include "gpio.h"
#include "SensorDataHandler.h"
#include "queue.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "ControllerAreaNetwork.h"
#include "periphAL.h"

int current_mode = NO_TRANSMISSION;

sensorPin right_sensor;
sensorPin back_sensor;
sensorPin left_sensor;
sensorPin front_sensor;

int sample_time = SAMPLE_TIME_DEFAULT;
int delay_command = SAMPLE_TIME_DEFAULT;
int units_time	= UNITS_TIME_DEFAULT;
can_frame_types_t	sentMessage;

int flag_counter = 0;

float gyro[3];
float accel[3];
float magn[3];

static void vReceiver(void *arg);
static void vSender(void *arg);
//static void vMainHandler(void *arg);
static TaskHandle_t transmitter_handler;
static TaskHandle_t receiver_handler;
//static TaskHandle_t main_handler;
static void floatToCAN(float data, can_frame_types_t *can_frame_types);
static void componentStatus(topic_can_id_t id, can_frame_types_t *can_frame_types);
static void setMode(int mode);

static void setMode(int mode){
	current_mode=mode;

	switch(current_mode){
	case NO_TRANSMISSION:
		vTaskSuspend(transmitter_handler);
		flag_counter = 0;
		break;

	case PUBLISHER:

		switch(units_time){
		case MILISECONDS:
			delay_command = sample_time;
			break;

		case SECONDS:
			delay_command = sample_time*1000;
			break;

		case MINUTES:
			delay_command = sample_time*60000;
			break;
		case MICROSECONDS:
			delay_command = sample_time;	//can't be arranged with current clock of 1000Hz
			break;
		}

		vTaskResume(transmitter_handler);
		break;

	case ONE_TIME_TRANSMISSION:
		vTaskSuspend(transmitter_handler);
		flag_counter = 0;
		vTaskResume(transmitter_handler);

		if(flag_counter>0){
			//run only once
			vTaskSuspend(transmitter_handler);
			flag_counter=0;
		}
		//handleACommand();
		break;
	}
}
//static void vMainHandler(void *pvParameters){
//
//	while(1){
//		switch(current_mode){
//		case NO_TRANSMISSION:
//			vTaskSuspend(transmitter_handler);
//			flag_counter = 0;
//			break;
//
//		case PUBLISHER:
//			vTaskResume(transmitter_handler);
//
//			break;
//
//		case ONE_TIME_TRANSMISSION:
//			vTaskSuspend(transmitter_handler);
//			flag_counter = 0;
//			vTaskResume(transmitter_handler);
//
//			if(flag_counter>0){
//				//run only once
//				vTaskSuspend(transmitter_handler);
//				flag_counter=0;
//			}
//			//handleACommand();
//			break;
//		}
//		vTaskDelay(100);
//	}
//}
void initHardware(void){
	initGyroscope();
	initAccelerometer();
	initMagnetometer();

	sentMessage.c_s_bit_id = SENSOR_DATA_BIT;
	sentMessage.dlc = 1;
	sentMessage.priority_id = PRIORITY_MEDIUM;

	right_sensor.num_trig = GPIO_PIN_2;
	right_sensor.grp_trig = (uint32_t) GPIOC;
	right_sensor.num_echo = GPIO_PIN_3;
	right_sensor.grp_echo = (uint32_t) GPIOC;

	back_sensor.num_trig = GPIO_PIN_10;
	back_sensor.grp_trig = (uint32_t) GPIOD;
	back_sensor.num_echo = GPIO_PIN_11;
	back_sensor.grp_echo = (uint32_t) GPIOD;

	left_sensor.num_trig = GPIO_PIN_10;
	left_sensor.grp_trig = (uint32_t) GPIOB;
	left_sensor.num_echo = GPIO_PIN_11;
	left_sensor.grp_echo = (uint32_t) GPIOB;

	front_sensor.num_trig = GPIO_PIN_2;
	front_sensor.grp_trig = (uint32_t) GPIOB;
	front_sensor.num_echo = GPIO_PIN_7;
	front_sensor.grp_echo = (uint32_t) GPIOE;

	//xTaskCreate(vMainHandler, "Main Handler", 200, NULL, tskIDLE_PRIORITY + 2, &main_handler);
	xTaskCreate(vReceiver, "Reception Handler", 200, NULL, tskIDLE_PRIORITY + 2, &receiver_handler);
	xTaskCreate(vSender, "Transimtter Handler", 200, NULL, tskIDLE_PRIORITY + 2, &transmitter_handler);

}


static void floatToCAN(float data, can_frame_types_t *can_frame_types){
		uint32_t aux = (uint32_t)(*(uint32_t*)&data);
		uint8_t datal = (uint8_t)(aux &  0x000000FF);
		uint8_t datah = (uint8_t)((aux & 0x0000FF00) >> 8);
		can_frame_types->data[0] = datah;
		can_frame_types->data[1] = datal;
}
static void componentStatus(topic_can_id_t id, can_frame_types_t *can_frame_types){
	if((errrx!=HAL_OK)||(errtx!=HAL_OK)){
		if((errrx==HAL_TIMEOUT)||(errtx==HAL_TIMEOUT)){
			can_frame_types->data[0] = HEALTH_CTRL_ID_COMP_STATUS;
			can_frame_types->data[1] = id;
			can_frame_types->data[2] = MODE1_TIMEOUT;
			can_transmit_data_frame_pd(can_frame_types);
		}
	}
	else{
		//IT IS OK!
	}
}
//void handleACommand(void){
static void vSender(void *pvParameters){
	while(1){
		if (receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_X ||
				receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_Y ||
				receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_Z){

			readGyroscope(gyro);
			if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_X){
				floatToCAN(gyro[0], &sentMessage);
				can_transmit_data_frame_pd(&sentMessage);
				componentStatus(INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_X, &sentMessage);
			}
			else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_Y){
				floatToCAN(gyro[1], &sentMessage);
				can_transmit_data_frame_pd(&sentMessage);
				componentStatus(INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_Y, &sentMessage);
			}
			else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_Z){
				floatToCAN(gyro[2], &sentMessage);
				can_transmit_data_frame_pd(&sentMessage);
				componentStatus(INTERNAL_ROBOT_CTRL_ID_GYROSCOPE_Z, &sentMessage);
			}
		}
		else if (receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_X ||
				receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_Y ||
				receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_Z){

			readAccelerometer(accel);
			if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_X){
				floatToCAN(accel[0], &sentMessage);
				can_transmit_data_frame_pd(&sentMessage);
				componentStatus(INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_X, &sentMessage);
			}
			else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_Y){
				floatToCAN(accel[1], &sentMessage);
				can_transmit_data_frame_pd(&sentMessage);
				componentStatus(INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_Y, &sentMessage);
			}
			else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_Z){
				floatToCAN(accel[2], &sentMessage);
				can_transmit_data_frame_pd(&sentMessage);
				componentStatus(INTERNAL_ROBOT_CTRL_ID_ACCELEROMETER_Z, &sentMessage);
			}
		}
		else if (receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_X ||
				receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_Y ||
				receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_Z){

			readMagnetometer(magn);
			if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_X){
				floatToCAN(magn[0], &sentMessage);
				can_transmit_data_frame_pd(&sentMessage);
				componentStatus(INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_X, &sentMessage);
			}
			else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_Y){
				floatToCAN(magn[1], &sentMessage);
				can_transmit_data_frame_pd(&sentMessage);
				componentStatus(INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_Y, &sentMessage);
			}
			else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_Z){
				floatToCAN(magn[2], &sentMessage);
				can_transmit_data_frame_pd(&sentMessage);
				componentStatus(INTERNAL_ROBOT_CTRL_ID_MAGNETOMETER_Z, &sentMessage);
			}
		}
		else if (receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_1 ||
				receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_2 ||
				receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_3 ||
				receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_4){

			if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_1){
				floatToCAN(readUltrasonic(&right_sensor), &sentMessage);
				can_transmit_data_frame_pd(&sentMessage);
				componentStatus(INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_1, &sentMessage);
			}
			else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_2){
				floatToCAN(readUltrasonic(&back_sensor), &sentMessage);
				can_transmit_data_frame_pd(&sentMessage);
				componentStatus(INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_2, &sentMessage);
			}
			else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_3){
				floatToCAN(readUltrasonic(&left_sensor), &sentMessage);
				can_transmit_data_frame_pd(&sentMessage);
				componentStatus(INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_3, &sentMessage);
			}
			else if(receivedMessage.topic_id == INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_4){
				floatToCAN(readUltrasonic(&front_sensor), &sentMessage);
				can_transmit_data_frame_pd(&sentMessage);
				componentStatus(INTERNAL_ROBOT_CTRL_ID_ULTRASONIC_SENSOR_4, &sentMessage);
			}
		}
		vTaskDelay(delay_command);
		flag_counter++;
		if(flag_counter>500){
			flag_counter = 0;
		}
	}
}
static void vReceiver(void *pvParameters){
	while(1){
		receivedMessage = can_receive_data_frame_pd();

		if(receivedMessage.c_s_bit_id ==SENSOR_DATA_BIT){		//sensor or command
			vTaskResume(transmitter_handler);
		}
		else if(receivedMessage.c_s_bit_id == COMMAND_DATA_BIT){	//command

			vTaskSuspend(transmitter_handler);

			if(receivedMessage.data[0] == NO_TRANSMISSION){
				setMode(NO_TRANSMISSION);
			}
			else if(receivedMessage.data[0]== PUBLISHER){
				if(receivedMessage.dlc > 1 ){
					if(receivedMessage.dlc==2){
						sample_time = receivedMessage.data[1];
						units_time = UNITS_TIME_DEFAULT;
					}
					else{
						sample_time = receivedMessage.data[1];
						units_time = receivedMessage.data[2];
					}
				}
				else{
					sample_time = SAMPLE_TIME_DEFAULT;
					units_time = UNITS_TIME_DEFAULT;
				}
				setMode(PUBLISHER);
			}
			else if(receivedMessage.data[0] == ONE_TIME_TRANSMISSION){
				setMode(ONE_TIME_TRANSMISSION);
			}
		}
		vTaskDelay(10);
	}
}
