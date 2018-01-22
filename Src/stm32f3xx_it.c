/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"
#include "cmsis_os.h"
#include "usart.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern int txDoneFlag;
/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
//void SysTick_Handler(void)
//{
//  /* USER CODE BEGIN SysTick_IRQn 0 */
//
//  /* USER CODE END SysTick_IRQn 0 */
//  osSystickHandler();
//  /* USER CODE BEGIN SysTick_IRQn 1 */
//
//  /* USER CODE END SysTick_IRQn 1 */
//}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/
/**
* @brief This function handles DMA1 channel6 global interrupt.
*/
		//RX
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */
	//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);	//LED8_ORANGE
	  if(DMA1->ISR & DMA_FLAG_TC6){
		  DMA1->IFCR = DMA_FLAG_TC6;


		  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
		  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);	//LED4_BLUE

		  DMA1->IFCR = DMA_FLAG_GL6 | DMA_FLAG_HT6 | DMA_FLAG_TC6 | DMA_FLAG_TE6;
	  }
  /* USER CODE END DMA1_Channel6_IRQn 0 */

  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */


  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel7 global interrupt.
*/

	//TX
void DMA1_Channel7_IRQHandler(void)
{
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */
	  if(DMA1->ISR & DMA_FLAG_TC7){
		  //HAL_NVIC_EnableIRQ(USART2_IRQn);
		  __HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);			//enable USART TC interrupt
		  DMA1->IFCR = DMA_FLAG_TC7;							//clear TC flags
		  __HAL_DMA_DISABLE_IT(&hdma_usart2_tx, DMA_IT_TC);		//disable DMA TC interrupt


		  DMA1->IFCR = DMA_FLAG_GL7 | DMA_FLAG_HT7 | DMA_FLAG_TC7 | DMA_FLAG_TE7;	//clear DMA pending bits

		  //txDoneFlag = 1;
		  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);	//LED8_ORANGE

		  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	  }
	//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);	//LED8_ORANGE
	//__HAL_GPIO_EXTI_GET_FLAG(DMA_FLAG_TC7);
  /* USER CODE END DMA1_Channel7_IRQn 0 */
	//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);	//LED8_ORANGE
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);	//LED8_ORANGE


  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

//  while(HAL_DMA_GetState(&hdma_usart2_tx) == HAL_DMA_STATE_BUSY){
//	  //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);	//LED8_ORANGE
//  }
  //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);	//LED8_ORANGE
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
  /* USER CODE END DMA1_Channel7_IRQn 1 */
}
/**
* @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
*/
void USART2_IRQHandler(void)
{

  /* USER CODE BEGIN USART2_IRQn 0 */
	if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC)){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);	//prepare to receive DE signal



		__HAL_UART_DISABLE_IT(&huart2, UART_IT_TC);		//disable TC interrupt
		__HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_TCF);	//clear TC pending bit
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);  	//LED9_BLUE

	}

	//HAL_NVIC_DisableIRQ(USART2_IRQn);
	//USART2->ISR
  /* USER CODE END USART2_IRQn 0 */
	 //HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);  	//LED9_BLUE
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles TIM7 global interrupt.
*/

//void TIM1_CC_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
//
//  /* USER CODE END TIM1_CC_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim1);
//  /* USER CODE BEGIN TIM1_CC_IRQn 1 */
//
//  /* USER CODE END TIM1_CC_IRQn 1 */
//}

void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
  //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);	//LED8_ORANGE
  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
