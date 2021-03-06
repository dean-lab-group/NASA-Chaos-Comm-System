/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
//This program WILL NOT WORK if not hooked up to oscillator and controller!!!!
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#define OFFSET 30
#include "ManchesterEncode.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static int TestArray[ONES_REPEATED + ZEROES_REPEATED]; 
	//To prevent transients on the oscillator, multiple copies of the same symbol must be sent in a row.
	//This sequence is actually 101010101010
int ArrLen = sizeof(TestArray) / sizeof(int);
int ArrInd = 0;
int NextSymbol = 85, CurrentSymbol = 85;
//85 = invalid input, prevents accidentaly entering loops

uint8_t ADC_data[1]; //for reading temp sensor
int ConvCplt = 0; //this is used to begin transmit function after ADC takes a reading
uint8_t ConvData[1];
uint8_t buffer1[100];
	

volatile int togglecheck = 1; // flag for when to update nextt transmitted symbol
char PolarityCheck = 'R'; //used to update current edge polarity (Rise or fall)
int stop_ADC = 1; 
int CaseNumber = 0; //used to (eventually) switch between data entry modes
int EdgeCount = 0; //counts edges detected to determine when to transmit next symbol

volatile uint32_t ADCConvertedValue; //holds last converted value
uint32_t LastValue = 5000; //invalid value to prevent invalid start

uint16_t CaptureIndex = 0;
uint32_t CapValue0 = 0;
uint32_t CapValue1 = 0;
uint32_t CapValue2 = 0;
uint32_t ValueDiff = 0;

uint32_t i = 0;

volatile uint32_t EntryModeFlag = 0;
volatile uint32_t StartFlag = 0;

uint8_t view1 = 0; // for viewing values parsed from ADC data
uint8_t view2 = 0;
uint8_t view3 = 0;
uint8_t view4 = 0;
uint8_t view5 = 0;
uint8_t view6 = 0;
uint8_t view7 = 0;
uint8_t view8 = 0;
int switch1 = 1;
int switch2 = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void TestCaseOutput(void); //output function for testcase mode
void ReadCustomData(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint16_t testindex = 0;
	uint16_t tmp_tot = ONES_REPEATED + ZEROES_REPEATED;
	for(testindex=0; testindex < ONES_REPEATED; testindex++){
		TestArray[testindex] = 1;		
	}
	for(testindex = ONES_REPEATED; testindex<tmp_tot; testindex++){
		TestArray[testindex] = 0;
	}
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);  //time base for adc sampling rate
	//HAL_ADC_Start_DMA(&hadc1, &ADCConvertedValue, sizeof(uint32_t));
  //HAL_TIM_OC_Start(&htim2,TIM_CHANNEL_1); 

	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_ADC_Start_IT(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_SET);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//		while(!StartFlag){
//		}
		//StartFlag = 0;
		switch(CaseNumber){
			case(0) :
				if (switch1 == 1)
				{
					HAL_ADC_Stop_IT(&hadc1); //stop the ADC to keep its callback and interrupt out of the way
					switch1 = 0;
					switch2 = 1;
				}
				while(togglecheck){ //if it's time to switch to next symbol THIS MAKES NO SENSE. 
					// If togglecheck is commented in line 205, test case works but cannot get out of this loop.
					TestCaseOutput(); //transmit
				}
				break;
			case(1) :
				if (switch2 == 1)
				{
					HAL_ADC_Start_IT(&hadc1); //Start the ADC to get temperature readings
					switch1 = 1;
					switch2 = 0;
				}
				ReadCustomData();
				
				break;
			case(2):
				break;
			default :
				break;
		}
	}	
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2624;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 63999;
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

  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 41;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 671;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 62499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
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

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TransistorSwitch_Pin */
  GPIO_InitStruct.Pin = TransistorSwitch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(TransistorSwitch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|TransistorSwitch_Pin, GPIO_PIN_RESET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void TestCaseOutput(void)
	{
	
	if(NextSymbol == 85){ //data validation for the initial transmission
		NextSymbol = TestArray[ArrInd];  //load the first transmission symbol in the array
		ArrInd++;
	}
	
	if(NextSymbol == 1){
		HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_SET);	
		//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	}else if(NextSymbol == 0){
		HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_RESET);
	}
	
	if((ArrInd + 1) <= (ArrLen))
		{ //prevent ArrInd going out of bounds
		CurrentSymbol = NextSymbol;
		NextSymbol = TestArray[ArrInd];
		ArrInd++;
		if(ArrInd == ArrLen)
			{
			ArrInd = 0;		
		}
	}
	togglecheck = 0; //clear flag
}

void ReadCustomData(void)
	{
			//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	//		static int p=0; //,i = 0;
			//uint8_t buffer[100]; //for inputting data
			//uint8_t recbuff[1]; //for inputting data
			uint8_t sendbuff[8];
			//p = ADC_data[0];
			//HAL_UART_Transmit(&huart2, buffer, p, 50);
		if (ConvCplt == 1)
		{
				ConvData[0] = ADC_data[0];
				for(i=0;i<8;i++)
				{
					if (ConvData[0]%2 == 1)
					{
						sendbuff[i] = 1;
					}
					else
					{
						sendbuff[i] = 0;
					}
				ConvData[0] = ConvData[0]/2;
				}					
				view1 = sendbuff[0];
				view2 = sendbuff[1];
				view3 = sendbuff[2];
				view4 = sendbuff[3];
				view5 = sendbuff[4];
				view6 = sendbuff[5];
				view7 = sendbuff[6];
				view8 = sendbuff[7];
	//		p = sprintf((char *)buffer, "Please enter in your own input:\n"); for entering data from putty/serial
	//		HAL_UART_Transmit(&huart2, buffer, p,50);
	//		HAL_UART_Receive(&huart2, recbuff, 2, 10000);
			//p = 0;
			//recbuff[0] = ;
	//		for(i=0;i<8;i++){
	//			sendbuff[i] = (recbuff[0] & (0x1 << i)); //this might parse the recbuff (which is stored as a single element) into 8 bits
	//			sendbuff[i] = sendbuff[i] >> i; //perhaps the best way to get ADC temp value to this state would be to create var ADCsendbuff[8] and do the same as is done to the value from UART
	//			//p++;
	//		}
					
		for(int k=0;k<7;k++)
				{
			//these two for loops create the start sequence of 1-1-0-0
				for(int z = 0; z<1; z++) //changed to z<1 for try of 1-0
				{
					for(int j=0;j<ONES_REPEATED;j++)
							{ //Flag a start to the sequence
								while(!togglecheck){}
								HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_SET);	
								//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
								togglecheck = 0;
							}
					//HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_RESET);
				}
				for(int z = 0; z<1; z++)
				{
					for(int j=0;j<ZEROES_REPEATED;j++) //changed to z<1 for 1-0
					{ //Flag a start to the sequence
								while(!togglecheck){}
								HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_RESET);	
								//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
								togglecheck = 0;
					}
					//HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_SET);
				 }				
	//this loop takes the 8 values in sendbuff and makes them into the bitstream to be sent	
if (k%2 == 0 && k<5)
{	
		for(i=0;i<8;i++)
		{
			if(sendbuff[i] == 1) //1 = 1-0
				{
				ManchesterOne();	
				ManchesterZero();
				if(k>4)
				{
				sendbuff[i] = 0;
				}
				}
				else if(sendbuff[i] == 0) //0 = 1-1-0
				{
				ManchesterOne();
				ManchesterOne();
				ManchesterZero();
				if(k>4)
				{
				sendbuff[i] = 0;	
				}					
				}
		}
	}
else //null delimiter thing
{
	for(i=0;i<8;i++)
	{
		ManchesterOne();
		ManchesterOne();
		ManchesterZero();
		//ManchesterZero();
	}
}
HAL_Delay(10);
	}
		ConvCplt = 0;
	}
		while(!togglecheck){ //set back to low
		}
		HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		togglecheck = 0; 
}
		

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1) //reads temperature sensor
{
		ADC_data[0] = HAL_ADC_GetValue(hadc1);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); //shows conversion happening every 2 seconds
		static int p1=0;
		p1 = sprintf((char *)buffer1, "%d\n0\n%d\n0\n%d\n0\n0\n",ADC_data[0],ADC_data[0],ADC_data[0]); // remember sprintf returns a length
		HAL_UART_Transmit(&huart2, buffer1, p1,50); //transmit data in buffer1 with length p1
		ConvCplt = 1;
}
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) //IGNORE THIS CALLBACK --UNUSED
//{
//	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); //pin toggle to ensure operation
//		
//	if(LastValue <= 4095){
//		if((LastValue < LOWEDGE) && (ADCConvertedValue > HIGHEDGE)){
//			EdgeCount++;
//			if(HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
//				{
//					Error_Handler();
//				}
//		}
//		
////		if(CurrentSymbol == 1){
////			if(EdgeCount >= HIGHCOUNT){
////				EdgeCount = 0;
////				togglecheck = 1;
////			}
////		}else if(CurrentSymbol == 0){
////			if(EdgeCount >= LOWCOUNT){
////				EdgeCount = 0;
////				togglecheck = 1;
////			}
////		}
//	}

//	LastValue = ADCConvertedValue; //store last state
//}

void HAL_TIM_IC_CaptureCallback( TIM_HandleTypeDef* htim ){ //this function writes the togglecheck back to 1 for the while loop in main
//this means the uc must have the oscillator folding mechanism on pin D12 to generate the 1010101010 pattern
	
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){ //check that the proper timer channel interrupted
		if((PolarityCheck == 'R') && (CaptureIndex == 0)){ //if the next edge is rising and the first capture
			CapValue0 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			CaptureIndex++; //increase index variable
			PolarityCheck = 'F'; //change capture polarity
		}else if(CaptureIndex == 1){ 
			CapValue1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			CaptureIndex++; 
			PolarityCheck = 'R';
		}else if(CaptureIndex == 2){
			CapValue0 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			CaptureIndex--; //bring back to CapInd = 1
			PolarityCheck = 'F';//change polarity
			togglecheck = 1; //now that ---__-- low pulse detected, begin next transmission
		}
		if(PolarityCheck == 'R'){ //change IC capture polarity
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
		}else if(PolarityCheck == 'F'){
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
	}
}

void HAL_TIM_OC_DelayElapsedCallback( TIM_HandleTypeDef* htim ){ 
	HAL_TIM_OC_Stop_IT(htim, TIM_CHANNEL_1);
	if(EntryModeFlag){
		CaseNumber = 1;
		EntryModeFlag = 0;
		StartFlag = 1;
	}else if(!CaseNumber){
		ArrInd = 0;
		ArrLen = sizeof(TestArray) / sizeof(int);
		//HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
		StartFlag = 1;
	}
}	

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	
	
//	if((htim4.Instance->CR1 && 0x0001) == 0){
//		HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
//	}else if((htim4.Instance->CR1 && 0x0001) == 1){
//		EntryModeFlag = 1;
//	}
	StartFlag = 1;
	CaseNumber++;
//	if(!CaseNumber){
//		HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
//		CaseNumber++;
//		ArrInd = 0;
//	}else{
//		CaseNumber = 0;
//		ArrLen = sizeof(TestArray) / sizeof(int);
//		ArrInd = 0;
//				HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
//	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart){
	
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
