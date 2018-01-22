/** @file ultrasonic.h
 *  @brief ultrasonic sensor evaluation
 *
 *  evaluates the ultrasonic sensor values and enqueues them
 *
 *  @created 2016.10.03
 *
 *  @author Heinrich Hubert Kosmann
 *  @author Kevin Liersch
 *
 *  @bug No known bugs.
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

/********************* includes ******************/

#include "queue.h"

/********************* defines ******************/

#define MODERX 0
#define MODETX 1
#define TRANSMIT_PROTOCOL_SIZE 6
#define RECEIVE_PROTOCOL_SIZE 8
#define ULTRASONICSENSOR_RIGHT_ADRESS 0x11
#define ULTRASONICSENSOR_BACK_ADRESS 0x12
#define ULTRASONICSENSOR_LEFT_ADRESS 0x13
#define ULTRASONICSENSOR_FRONT_ADRESS 0x14


/********************* struct/enum/union ******************/
typedef struct
{
  uint16_t right;
  uint16_t back;
  uint16_t left;
  uint16_t front;
  double theta;

}ultrasonic_Data_t;
/********************* global Variables ******************/
QueueHandle_t ultraSonicQueue;
/********************* global prototypes ******************/

void ultrasonic_Init();

#endif /* ULTRASONIC_H_ */
