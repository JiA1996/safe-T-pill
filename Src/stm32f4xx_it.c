/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//extern TIM_HandleTypeDef htim3;
int pwm_flag = 1;
int tim_flag = 1;
int ti_flag = 6;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart4;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim3;
int count = 0;

//extern UART_HandleTypeDef huart4;
//extern UART_HandleTypeDef huart2;
//char update_request[10] = "update001\n";
extern SPI_HandleTypeDef hspi2;
extern int pill_list[10];
extern int empty_list[10];
//extern char rxdata[400];
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
	//HAL_UART_Transmit(&huart4, update_request, 10, 1000);
	//memset(rxdata, 0, sizeof(rxdata));
	//HAL_UART_Receive_IT(&huart4, rxdata, 200);
	//HAL_UART_Transmit(&huart2, rxdata, 200, 1000);
  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  uint8_t led_index[20] = {
		  0xff, 0xfe,//1st led
		  0xff, 0xfd,//2nd led
		  0xff, 0xfb,//3rd led
		  0xff, 0xf7,//4th led
		  0xff, 0xef,//5th led
		  0xff, 0xdf,//6th led
		  0xff, 0xbf,//7th led
		  0xff, 0x7f,//8th led
		  0xfe, 0xff,//9th led
		  0xfe, 0xfd};//10th d
  uint8_t value_table[11] = {0xff, 0xeb, 0x4c, 0x49, 0x2b, 0x19, 0x18, 0xcb, 0x08, 0x0b, 0x1c};
  	  	  	  	  	  	  	//empty 1     2     3     4     5     6     7     8     9////
  if (count < 60){
	  if (tim_flag == 1) {
		  if (empty_list[0] == 1){
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + 10, 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }else {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + 10, 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }
		  tim_flag = 2;
		  count += 1;
	  } else if (tim_flag == 2) {
		  if (empty_list[1] == 1){
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 2, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + 10, 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }else {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 2, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + 10, 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }
		  tim_flag = 3;
		  count += 1;
	  } else if (tim_flag == 3) {
		  if (empty_list[2] == 1){
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 4, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + 10, 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }else {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 4, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + pill_list[2], 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }
		  tim_flag = 4;
		  count += 1;
	  } else if (tim_flag == 4) {
		  if (empty_list[3] == 1){
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 6, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + 10, 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }else {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 6, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + pill_list[3], 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }
		  tim_flag = 5;
		  count += 1;
	  } else if (tim_flag == 5) {
		  if (empty_list[4] == 1){
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 8, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + 10, 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }else {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 8, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + pill_list[4], 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }
		  tim_flag = 1;
		  count += 1;
	  }
  } else if (count >= 60 && count < 120) {
	  if (ti_flag == 6)  {
		  if (empty_list[5] == 1){
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 10, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + 10, 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }else {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 10, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + pill_list[5], 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }
		  ti_flag = 7;
		  count += 1;
	  } else if (ti_flag == 7) {
		  if (empty_list[6] == 1){
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 12, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + 10, 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }else {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 12, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + pill_list[6], 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }
		  ti_flag = 8;
		  count += 1;
	  } else if (ti_flag == 8) {
		  if (empty_list[7] == 1){
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 14, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + 10, 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }else {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 14, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + 10, 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }
		  ti_flag = 9;
		  count += 1;
	  } else if (ti_flag == 9) {
		  if (empty_list[8] == 1){
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 16, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + 10, 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }else {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 16, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + 10, 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }
		  ti_flag = 10;
		  count += 1;
	  } else if (ti_flag == 10) {
		  if (empty_list[8] == 1){
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 18, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + 10, 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }else {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi2, led_index + 18, 2, 100);
			  HAL_SPI_Transmit(&hspi2, value_table + 10, 1, 100);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		  }
		  ti_flag = 6;
		  count += 1;
	  }
  } else {
	  count = 0;
  }

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
  if (pwm_flag == 1){
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	  pwm_flag = 2;
  } else if (pwm_flag == 2){
	  pwm_flag = 1;
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
	  HAL_TIM_Base_Stop_IT(&htim4);
  }
  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
