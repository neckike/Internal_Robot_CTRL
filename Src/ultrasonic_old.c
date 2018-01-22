/** @file ultrasonic.c
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

/********************* includes ******************/

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "stdlib.h"
#include "task.h"
#include "ultrasonic.h"
#include "Trace.h"
#include "usart.h"
#include "gpio.h"
#include "queue.h"
//#include "encoder.h"
//#include "wt41a.h"	//bluetooth

/********************* defines ******************/

/********************* struct/enum/union ******************/

typedef enum
{
  ultrasonic_trigger = 0x01,
  ultrasonic_distance = 0x02,
  ultrasonic_temperature = 0x03,
} ultrasonicMode_e;

struct Adress_t
{
  uint8_t right;
  uint8_t back;
  uint8_t left;
  uint8_t front;

}Adress_t;

/********************* local Variables ******************/

/********************* local prototypes ******************/

static void vUltrasonic(void *arg);
static TaskHandle_t ultrasonic_task_handle;
void getSensorData(struct Adress_t sensorAdresses, ultrasonicMode_e dataMode);
HAL_StatusTypeDef transmitSensor(uint8_t sensorAdress, ultrasonicMode_e dataMode);
uint16_t receiveSensor(uint8_t sensorAdress);
void digitalWrite(uint8_t mode);
static inline void waitForDMAForReceiveComplete(const TickType_t xTicksToDelay);
static inline void waitForDMAForTransmitComplete(const TickType_t xTicksToDelay);


int txDoneFlag = 0;

/********************* global functions ******************/

/** @brief creates the task
 *
 *  @param void
 *  @return void
 */
void ultrasonic_Init()
{
//  ultraSonicQueue = xQueueCreate( 1, sizeof(ultrasonic_Data_t ) );
//  if(ultraSonicQueue == NULL)
//  {
//    trace_printf("Queue konnte nicht erstellt werden\n");
//    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);		//LED10_RED		SOUTH
//  }
//
    DMA_Sensor_RxComplete = 0;
    DMA_Sensor_TxComplete = 0;

  xTaskCreate(vUltrasonic, "Ultrasonic", 200, NULL, tskIDLE_PRIORITY + 2, &ultrasonic_task_handle);
  trace_puts("Ultrasonic_Init abgeschlossen\n");

  //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);			//LED7_GREEN	EAST
}

/********************* local functions ******************/

/** @brief ultrasonic task
 *
 *  gets the values of the sensors
 *
 *  @param arg
 *  @return void
 */

static void vUltrasonic(void *arg)
{
  //TickType_t t;
  //TickType_t xLastWakeTime = xTaskGetTickCount();
  struct Adress_t sensorAdresses;
  sensorAdresses.right = ULTRASONICSENSOR_RIGHT_ADRESS;
  sensorAdresses.back = ULTRASONICSENSOR_BACK_ADRESS;
  sensorAdresses.left = ULTRASONICSENSOR_LEFT_ADRESS;
  sensorAdresses.front = ULTRASONICSENSOR_FRONT_ADRESS;

  //double thetaTmp = 0;

  //int aktThetaCalc = 0;
  //int thetaTmpCalc = 0;
  //int thetaDiff = 0;
  uint8_t i, j;
    uint8_t protocol[TRANSMIT_PROTOCOL_SIZE];
    uint8_t sprotocol[7];
    uint8_t sprotocol2[7];
    uint8_t rep[RECEIVE_PROTOCOL_SIZE];			//maybe volatile?
    uint8_t rep2[RECEIVE_PROTOCOL_SIZE];			//maybe volatile?
    HAL_StatusTypeDef status, status2;

    protocol[0] = 0x55 ;
    protocol[1] = 0xaa ;
    protocol[2] = sensorAdresses.right;
    protocol[3] = 0 ;
    protocol[4] =  ultrasonic_trigger;
    protocol[5] = 0;
    ultrasonic_Data_t sensorData = {0,0,0,0,0};
    uint16_t test=0;

    //uint8_t protocol[RECEIVE_PROTOCOL_SIZE];// = {0};
    uint8_t checksum = 0;
    //uint8_t i;

   sprotocol[0] = 0x55 ;
   sprotocol[1] = 0xaa ;
   sprotocol[2] = 0xab;
   sprotocol[3] = 0x01 ;
   sprotocol[4] = 0x55;
   sprotocol[5] = 0x11;
   sprotocol[6] = 0;
/*
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
   	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
   	for(i= 0; i < 7; i++){
   		sprotocol[6] += sprotocol[i];
   	}
   	__HAL_UART_FLUSH_DRREGISTER(&huart2);
   	if(HAL_UART_Transmit(&huart2,sprotocol,7,50)==HAL_OK ){
   		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);	//LED5_ORANGE
   	}
   	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
   	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
   	__HAL_UART_FLUSH_DRREGISTER(&huart2);
   	if(HAL_UART_Receive(&huart2, sprotocol, 7, 50) == HAL_TIMEOUT){
   		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);  	//LED9_BLUE
   	}
   	if(sprotocol[5] == 0x01){
   		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14); 		//LED8_ORANGE
   	}
   	vTaskDelay(500);
   	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12,GPIO_PIN_RESET);  	//LED9_BLUE
   	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10,GPIO_PIN_RESET);  	//LED5_ORANGE*/
  while (1){




//    //getSensorData(sensorAdresses, ultrasonic_distance);
//    //vTaskDelayUntil(&xLastWakeTime, 125);
//	    protocol[0] = 0x55 ;
//	    protocol[1] = 0xaa ;
//	    protocol[2] = sensorAdresses.right;
//	    protocol[3] = 0 ;
//	    protocol[4] = ultrasonic_trigger;
//	    protocol[5] = 0;
//    //HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);	//LED5_ORANGE

	    //__HAL_UART_FLUSH_DRREGISTER(&huart2);
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	    status = transmitSensor(sensorAdresses.right, ultrasonic_trigger);

//      for(i= 0; i < TRANSMIT_PROTOCOL_SIZE-1; i++)
//      {
//        protocol[5] += protocol[i];
//      }
//      //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//      //__HAL_UART_FLUSH_DRREGISTER(&huart2);
//      status = HAL_UART_Transmit_DMA(&huart2,protocol,TRANSMIT_PROTOCOL_SIZE);

//		if(status==HAL_OK){
//			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15);	//LED6_GREEN	WEST
//		}
	    while(status == HAL_BUSY){}
	    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15);	//LED6_GREEN	WEST

		/*******************************************HERE MIGHT BE THE PROBLEM, DMA NOT WAITING UNTIL IT IS COMPLETE, SENDING ANOTHER ONE*********************/



		//while(HAL_DMA_GetState(&hdma_usart2_tx) != HAL_DMA_STATE_READY){}

		//vTaskDelay(40);
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		//__HAL_UART_FLUSH_DRREGISTER(&huart2);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
		transmitSensor(sensorAdresses.right, ultrasonic_distance);

//      //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
//
//	    protocol[0] = 0x55 ;
//	    protocol[1] = 0xaa ;
//	    protocol[2] = sensorAdresses.right;
//	    protocol[3] = 0;
//	    protocol[4] = ultrasonic_distance;
//	    protocol[5] = 0;
//
//        for(i= 0; i < TRANSMIT_PROTOCOL_SIZE-1; i++)
//        {
//          protocol[5] += protocol[i];
//        }
//        //__HAL_UART_FLUSH_DRREGISTER(&huart2);
//       //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//        //HAL_UART_DMAStop(&huart2);
//
//        //HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);	//LED7_GREEN	EAST
//        __HAL_UART_FLUSH_DRREGISTER(&huart2);
//
//
//        status = HAL_UART_Transmit_DMA(&huart2, protocol, TRANSMIT_PROTOCOL_SIZE);

//        if(status==HAL_OK){
//      	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);	//LED7_GREEN	EAST
//        }

        while(status == HAL_BUSY){}



        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);	//LED7_GREEN	EAST

        //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        vTaskDelay(40);
//RECEIVE		Here is where the problems begin





//        status = HAL_UART_Receive(&huart2, rep, RECEIVE_PROTOCOL_SIZE, 70);
//        while(HAL_UART_STATE_BUSY_RX){
//                	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);	//LED5_ORANGE
//                	//Error_Handler();
//        }
//        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);	//LED5_ORANGE
//
        HAL_UART_DMAStop(&huart2);

        status = HAL_UART_Receive_DMA(&huart2, rep, RECEIVE_PROTOCOL_SIZE);

        if(status != HAL_OK){
        	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);	//LED5_ORANGE
        	Error_Handler();
        }
        //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);	//LED5_ORANGE
//        while(!DMA_IT_TC){
//        	band++;
//        }
//        if(band==0){
//        	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);	//LED5_ORANGE
//        	band=0;
//        }
        //HAL_DMA_IRQHandler(&hdma_usart2_rx);
        //HAL_DMA_Start_IT(&hdma_usart2_rx, &rep, &rep2, RECEIVE_PROTOCOL_SIZE);
        //status2 = HAL_DMA_PollForTransfer(&hdma_usart2_rx, HAL_DMA_FULL_TRANSFER, 50);
        //vTaskDelay(50);
        //HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);	//LED4_BLUE
        //UART_DMA_RX_ENABLE


//        status = HAL_UART_Receive(&huart2, rep, RECEIVE_PROTOCOL_SIZE, 100);
//
//        if(status == HAL_TIMEOUT){
//        	c
//        }

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

        for(j= 0; j < RECEIVE_PROTOCOL_SIZE-1; j++)
        {
          checksum += rep[j];

        }

        if(checksum != rep[RECEIVE_PROTOCOL_SIZE-1])
        //if(checksum == 0)
        {
        	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);		//LED10_RED		SOUTH	//BAAAAAAD
        }

        else{
        	test = (rep[5]<<8) + rep[6];//sensorData.right= (rep[5]<<8) + rep [6];
        }


								if(rep[5]!=0){
									//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);  	//LED9_BLUE
								}


        /*if(rep[RECEIVE_PROTOCOL_SIZE-1] == 0){
        	test = (rep[5]<<8) + rep[6];//sensorData.right= (rep[5]<<8) + rep [6];
        }
        else{
        	test =0xffff;
        }*/
       //xQueueOverwrite( ultraSonicQueue, &sensorData );

       if(test>20000){
    	   HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);  	//LED9_BLUE
       }
       else if((test<20000)&&(test!=0)){
    	   HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14); 		//LED8_ORANGE
       }
       else if(test==0){
    	   HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);  	//LED3_RED		NORTH
       }
       else if(test == 0xffff){
    	   HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);	//LED4_BLUE
       }
    vTaskDelay(300);
  }
}


/** @brief gets sensor data
 *
 *  1 Cycle 68ms
 *
 *  @param sensorAdresses
 *  @param dataMode trigger, distance, temperature
 *
 *  @return void
 */
void getSensorData(struct Adress_t sensorAdresses, ultrasonicMode_e dataMode)
{
  //Pose_t aktPoseRKS = {0,0,0};
  //int i;
  ultrasonic_Data_t sensorData = {0,0,0,0,0};

  transmitSensor(sensorAdresses.right, ultrasonic_trigger);// triggern

  transmitSensor(sensorAdresses.back, ultrasonic_trigger);// triggern

  transmitSensor(sensorAdresses.left, ultrasonic_trigger);// triggern

  transmitSensor(sensorAdresses.front, ultrasonic_trigger);// triggern

  //vTaskDelay(125);
  //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);			//LED7_GREEN	EAST
  transmitSensor(sensorAdresses.right, dataMode);
  sensorData.right = receiveSensor(sensorAdresses.right);

  transmitSensor(sensorAdresses.back, dataMode);
  sensorData.back = receiveSensor(sensorAdresses.back);

  transmitSensor(sensorAdresses.left, dataMode);
  sensorData.left = receiveSensor(sensorAdresses.left);

  transmitSensor(sensorAdresses.front, dataMode);
  sensorData.front = receiveSensor(sensorAdresses.front);
  //trace_printf("\n");

  //encoders to work uncomment.
  //encoder_getOdo(&aktPoseRKS.x, &aktPoseRKS.y, &aktPoseRKS.theta);
  //sensorData.theta = aktPoseRKS.theta;

  //if(((int)(aktPoseRKS.theta /M_PI*180.0)+180)%45 > 42 || ((int)(aktPoseRKS.theta /M_PI*180.0)+180)%45 < 2)

//  if(sensorData.right != 65535) sensorData.right = 0;
//  if(sensorData.left == 65535) sensorData.left = 0;
//  if(sensorData.front == 65535) sensorData.front = 0;
//  if(sensorData.back == 65535) sensorData.back = 0;

  //if(sensorData.right != 65535 && sensorData.left != 65535 && sensorData.front != 65535 && sensorData.back != 65535)





  //   xQueueOverwrite( ultraSonicQueue, &sensorData );



    if(sensorData.right >20000){     									//correct info= blue led
      	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);	//LED8_ORANGE

      	  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
      	  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
      	  //vTaskDelay(50);
        }
    else if(sensorData.right ==0){
    	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15);	//LED6_GREEN	WEST
    }
   /* if(sensorData.front != 65534){     									//correct info= blue led
  	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);	//LED8_ORANGE

  	  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
  	  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
  	  //vTaskDelay(50);
    }
    else if(sensorData.front == 65534){  								//corrupted = red led
  	  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
  	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15);	//LED6_GREEN	WEST
  	  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
  	  //vTaskDelay(50);
    }*/

//    wt41DebugData.front = sensorData.front;
//    wt41DebugData.back = sensorData.back;
//    wt41DebugData.left = sensorData.left;
//    wt41DebugData.right = sensorData.right;

//    wt41DebugData.front = sensorData.front;
//    wt41DebugData.right =   sensorData.right;
//    wt41DebugData.back = sensorData.back;
//    wt41DebugData.left = sensorData.left;
  //trace_printf("Sensorvalues: %d\t%d\t%d\t%d\n", sensorData.right, sensorData.back, sensorData.left, sensorData.front);
}

/** @brief transmits the given command to the sensor
 *
 *  @param sensorAdress
 *  @param dataMode trigger, distance, temperature
 *  @return void
 */

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//    txDoneFlag = 1;
//
//}


HAL_StatusTypeDef transmitSensor(uint8_t sensorAdress, ultrasonicMode_e dataMode)
{
  uint8_t i;
  uint8_t protocol[TRANSMIT_PROTOCOL_SIZE];
  HAL_StatusTypeDef status;

  protocol[0] = 0x55 ;
  protocol[1] = 0xaa ;
  protocol[2] = sensorAdress ;
  protocol[3] = 0 ;
  protocol[4] = dataMode;
  protocol[5] = 0;

  for(i= 0; i < TRANSMIT_PROTOCOL_SIZE-1; i++)
  {
    protocol[5] += protocol[i];
  }

  //digitalWrite(MODETX);


  //__HAL_UART_FLUSH_DRREGISTER(&huart2);


  status = HAL_UART_Transmit_DMA(&huart2,protocol,TRANSMIT_PROTOCOL_SIZE);
//  if(txDoneFlag==0){
//	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);  	//LED3_RED		NORTH
//	  vTaskDelay(10);
//  }
//  txDoneFlag = 0;
  //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);  	//LED3_RED		NORTH


  //status = HAL_UART_Transmit(&huart2,protocol,TRANSMIT_PROTOCOL_SIZE, 50);

  //waitForDMAForTransmitComplete(5);

  //digitalWrite(MODERX);

  return status;
}

/** @brief Reads out the sensor values.
 *
 *  @param sensorAdress
 *  @return sensor value if corrupted 0xffff
 */


uint16_t receiveSensor(uint8_t sensorAdress)
{
  uint8_t protocol[RECEIVE_PROTOCOL_SIZE];// = {0};
  uint8_t checksum = 0;
  uint8_t i;

  __HAL_UART_FLUSH_DRREGISTER(&huart2);
  HAL_UART_Receive_DMA(&huart2, protocol, RECEIVE_PROTOCOL_SIZE);

  waitForDMAForReceiveComplete(10);

  for(i= 0; i < RECEIVE_PROTOCOL_SIZE-1; i++)
  {
    checksum += protocol[i];

  }

  if(checksum != protocol[RECEIVE_PROTOCOL_SIZE-1])
  {

    return 0xFFFE;
  }
  //for(i = 0;i <8;i++)
    //trace_printf("%d %d %d %d %d %d %d %d \n",protocol[0],protocol[1],protocol[2],protocol[3],protocol[4],protocol[5],protocol[6],protocol[7]);
    //vTaskDelay(125);
 // trace_printf("\n");
  return (protocol[5]<<8) + protocol[6];
}

/** @brief Activates RXD or TXD of RS485
 *
 *  @param mode RXD or TXD
 *  @return void
 */
void digitalWrite(uint8_t mode)
{
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET && mode == MODETX)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
  } else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)
      == GPIO_PIN_SET&& mode == MODERX)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
  }
}


/** @brief Waits for DMA RX to finish
 *
 *
 *  @param xTicksToDelay
 *  @return void
 */
static inline void waitForDMAForReceiveComplete(const TickType_t xTicksToDelay)
{
  uint8_t timeout = 10;
  while(!DMA_Sensor_RxComplete && timeout)
  {

    //vTaskDelay(xTicksToDelay);
    timeout--;
  }
  if(!timeout)
  {
    HAL_UART_DMAStop(&huart2);
    trace_printf("timeout receive\n");
  }
  DMA_Sensor_RxComplete = 0;
}

/** @brief Waits for DMA TX to finish
 *
 *
 *  @param xTicksToDelay
 *  @return void
 */
static inline void waitForDMAForTransmitComplete(const TickType_t xTicksToDelay)
{
  uint8_t timeout = 5;
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);			//LED7_GREEN	EAST
  while(!DMA_Sensor_TxComplete && timeout)
  {
	//vTaskDelay(xTicksToDelay);
    timeout--;
  }
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);			//LED7_GREEN	EAST
  if(!timeout)
  {
    HAL_UART_DMAStop(&huart2);
    trace_printf("timeout transmit\n");
  }
  DMA_Sensor_TxComplete = 0;
}
