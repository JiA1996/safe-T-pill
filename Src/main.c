/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
int tim_flag = 1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

char time_time[5] = "time\n";
int minutes_match = 4;//used to check real time
int hours_match = 15;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  sDate.Date = 27;
  sDate.Month = RTC_MONTH_FEBRUARY;
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Year = 19;
  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  char email_request_fill[14];
  email_request_fill[10] = 'f';
  email_request_fill[11] = '1';
  email_request_fill[12] = '2';
  email_request_fill[13] = '3';
  char* email_request_time[10] = {"0t123\n", "1t123\n", "2t123\n", "3t123\n", "4t123\n", "5t123\n", "6t123\n", "7t123\n", "8t123\n", "9t123\n"};
  ///////////////////////////////////////////////////email_boxindex_time_to_take-pill
  //char e0t[6] = "0t123\n", e1t[6] = "1t123\n", e2t[6] = "2t123\n", e3t[6] = "3t123\n", e4t[6] = "4t123\n",
//		  e5t[6] = "5t123\n", e6t[6] = "6t123\n", e7t[6] = "7t123\n", e8t[6] = "8t123\n", e9t[6] = "9t123\n";
  ///////////////////////////////////////////////////email_boxindex_need_update
  char update_request[7] = "update\n";
  char time_to_take_pill[30] = "time to take pill!\n";
  //int update_timer = 1;
  int * pill_num[10];
  int * hour[10];
  int * minute[10];

  //int buzzer = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim2);
  char rxData[400];
  while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /* set time */
	  HAL_UART_Transmit(&huart4, time_time, sizeof(time_time), 1000);
	  memset(rxData, 0, sizeof(rxData));
	  HAL_UART_Receive(&huart4, rxData, sizeof(rxData), 10000);
	  HAL_UART_Transmit(&huart2, rxData, sizeof(rxData), 1000);//mcu->tera
	  set_time(rxData);


	  /* send update request and parse dosage info*/
	  memset(rxData, 0, sizeof(rxData));
	  HAL_UART_Transmit(&huart4, update_request, 7, 1000);
	  HAL_UART_Receive(&huart4, rxData, sizeof(rxData), 12000);
	  HAL_UART_Transmit(&huart2, rxData, sizeof(rxData), 1000);//mcu->tera
	  char dosage_info[400];
	  strncpy(dosage_info, rxData+12, 400);
	  parse(&pill_num, &hour, &minute, dosage_info);


	  /* sensor check */
	  int * sensors = check_sensors();
	  for(int a = 0; a < 10; a++){//check sensor
		  if(sensors[a] == 1){//change to check multiple segments
			  email_request_fill[a] = a + '0';
		  }else{
			  email_request_fill[a] = 'n';
		  }
	  }
	  memset(rxData, 0, sizeof(rxData));
	  HAL_UART_Transmit(&huart4, email_request_fill, 20, 1000);
	  HAL_UART_Receive(&huart4, rxData, sizeof(rxData), 10000);

  	for(int a = 0; a < 10; a++){//loop through segments 0-9
  		for (int b = 0; b < 5; b++)
  		if((minute[a][b] == sTime.Minutes) && (hour[a][b] == sTime.Hours)){
  			HAL_UART_Transmit(&huart2, time_to_take_pill, sizeof(time_to_take_pill), 10000);

  			HAL_UART_Transmit(&huart2, email_request_time[a], 6, 10000);//mcu->esp
  			char tmp_buffer[4];
  			HAL_UART_Transmit(&huart2,tmp_buffer,sprintf(tmp_buffer,"p%d\n",pill_num[a][b]), 10000);
  		}
  	}

  	//memset(rxData, 0, sizeof(rxData));
  	HAL_Delay(5000);
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /**Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /**Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 12;
  sTime.Minutes = 10;
  sTime.Seconds = 30;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_FEBRUARY;
  sDate.Date = 27;
  sDate.Year = 19;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  __TIM2_CLK_ENABLE();
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  //htim2.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD11 PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  UNUSED(htim);
  //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
  ///////////////////////1    eb    2    4c  3    49    4   2b   5    59     6     18    7			8			9=256
  uint8_t dataOut[30] = {0x00,0x01,0xeb,0x00,0x02,0x4c,0x00,0x04,0x49,0x00,0x08,0x2b,0x00,0x10,0x59,0x00, 0x20, 0x18,0x00, 0x40, 0xcb,0x00, 0x80, 0x08, 0x01, 0x00, 0x0b, 0x02, 0x00, 0x1c}; //code for data output, second byte is for
  uint8_t led_index[10] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x100, 0x200};
  uint8_t truth_table[10] = {0x1c, 0xeb, 0x4c, 0x49, 0x3b, 0x59, 0x18, 0xcb, 0x08, 0x0b};

  if (tim_flag == 1) {
  	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
  	HAL_SPI_Transmit(&hspi2,dataOut, 1, 100);
  	HAL_SPI_Transmit(&hspi2,dataOut + 1, 1, 100);
  	HAL_SPI_Transmit(&hspi2,dataOut + 2, 1, 100);
  	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
  	tim_flag = 2;
  }
  else if (tim_flag == 2) {

  	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
  	HAL_SPI_Transmit(&hspi2,dataOut + 3, 1, 100);

  	HAL_SPI_Transmit(&hspi2,dataOut + 4, 1, 100);
  	HAL_SPI_Transmit(&hspi2,dataOut + 5, 1, 100);
  	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
  	tim_flag = 3;
  }
  else if (tim_flag == 3) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,dataOut + 6, 1, 100);
	HAL_SPI_Transmit(&hspi2,dataOut + 7, 1, 100);
	HAL_SPI_Transmit(&hspi2,dataOut + 8, 1, 100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
	tim_flag = 1;
  }
  /*else if (tim_flag == 9) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,dataOut+24, 1, 100);
	HAL_SPI_Transmit(&hspi2,dataOut+25, 1, 100);
	HAL_SPI_Transmit(&hspi2,dataOut+26, 1, 100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
	tim_flag = 1;
  }*/

}


int * check_sensors(){
    int sensor0 = 0, sensor1 = 0, sensor2 = 0, sensor3 = 0, sensor4 = 0, sensor5 = 0, sensor6 = 0, sensor7 = 0, sensor8 = 0, sensor9 = 0;//index for each sensor
	sensor0 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
	sensor1 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
	sensor2 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
	sensor3 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
	sensor4 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
	sensor5 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
	sensor6 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
	sensor7 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
	sensor8 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
	sensor9 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
	static int arr[10];
	arr[0] = sensor0; arr[1] = sensor1; arr[2] = sensor2; arr[3] = sensor3; arr[4] = sensor4;
	arr[5] = sensor5; arr[6] = sensor6; arr[7] = sensor7; arr[8] = sensor8; arr[9] = sensor9;
	return arr;
}

void set_time(char* rxData)
{
	char tmph[2]; char tmpm[2]; char tmps[2];
	strncpy(tmph, rxData+12,2);
	int update_hour = atoi(tmph);
	strncpy(tmpm, rxData+14,2);
	int update_minute = atoi(tmpm);
	strncpy(tmps, rxData+16,2);
	int update_second = atoi(tmps);
	char time_test[10] = "mcutime:";
	HAL_UART_Transmit(&huart2, time_test, sizeof(time_test), 100);//hour
	HAL_UART_Transmit(&huart2, tmph, sizeof(tmph), 100);//hour
	HAL_UART_Transmit(&huart2, tmpm, sizeof(tmpm), 100);//minute
	HAL_UART_Transmit(&huart2, tmps, sizeof(tmps), 100);//second
	sTime.Hours = update_hour;
	sTime.Minutes = update_minute;
	sTime.Seconds = update_second;
	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
}

void parse(int ** pill_num, int ** hour, int ** minute, char * dosage_info)
{
    int i = 0, j = 0;
    for(i=0; i < 10; i++) {
        pill_num[i] = (int *)malloc(5 * sizeof(int));
        hour[i] = (int *)malloc(5 * sizeof(int));
        minute[i] = (int *)malloc(5 * sizeof(int));
    }

    //char * info2 = "p0t0830n2t1120n3t2045n2\n";
    //char * info = "p0t0830n2t1120n3t2045n2p1t0920n1t1230n1t2210n5p2t1015n6t2305n6\n";
    int start=0;//cursor c1
    int end=0;//cursor c2
    int seg_id=0;//index of the segment on pill box
    int ind = 0;//index indicates number of times to take pill
    //get dosage_info for each segment
    while (dosage_info[start]!='\n'){
        if(dosage_info[start]=='p') {
            end=start + 1;
            while(dosage_info[end] != '\n'){
                if(dosage_info[end] == 'p'){
                    break;
                }else{
                    end++;
                }
            }
            //printf("%d, %d\n",start,end);
            int length = end - start;
            char* tmpinfo = malloc(sizeof(char) * length);//information for one segment info
            strncpy(tmpinfo, dosage_info+start,end-start);
            //printf("%s\n",tmpinfo);
            int sub_cursor = 3;
            char boxid [1]; strncpy(boxid, tmpinfo+1,1);
            seg_id = atoi(boxid);
            ind=0;
            while(sub_cursor < length){//store dosage_information in stm32
                char* tmp_hour = malloc(sizeof(char)*2);
                char* tmp_min = malloc(sizeof(char)*2);
                char* tmp_num = malloc(sizeof(char)*1);
                strncpy(tmp_hour, tmpinfo + sub_cursor, 2);
                char tmp_buffer[10];

                sub_cursor += 2;
                hour[seg_id][ind] = atoi(tmp_hour);
                strncpy(tmp_min, tmpinfo + sub_cursor, 2);
                sub_cursor += 3;
                minute[seg_id][ind] = atoi(tmp_min);
                strncpy(tmp_num, tmpinfo + sub_cursor, 1);
                sub_cursor += 2;
                pill_num[seg_id][ind] = atoi(tmp_num);
                ind ++;
                free(tmp_hour); free(tmp_min); free(tmp_num);
            }
             while(ind < 5){//generate invalid values
                pill_num[seg_id][ind]=0;
                hour[seg_id][ind]=77;
                minute[seg_id][ind]=77;
                ind ++;
            }
            free(tmpinfo);
            start=end;
        }
        else{start ++;}
    }
    ind = 0;
    seg_id++;
    while(seg_id < 10){
        for(ind = 0; ind < 5; ind ++){
            pill_num[seg_id][ind]=0;
            hour[seg_id][ind]=77;
            minute[seg_id][ind]=77;
        }
        seg_id ++;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
