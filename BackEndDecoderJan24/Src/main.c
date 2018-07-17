/**
	******************************************************************************
	* File Name					: main.c
	* Description				: Main program body
	******************************************************************************
	** This notice applies to any and all portions of this file
	* that are not between comment pairs USER CODE BEGIN and
	* USER CODE END. Other portions of this file, whether 
	* inserted by the user or by software development tools
	* are owned by their respective copyright owners.
	*
	* COPYRIGHT(c) 2017 STMicroelectronics
	*
	* Redistribution and use in source and binary forms, with or without modification,
	* are permitted provided that the following conditions are met:
	*	 1. Redistributions of source code must retain the above copyright notice,
	*			this list of conditions and the following disclaimer.
	*	 2. Redistributions in binary form must reproduce the above copyright notice,
	*			this list of conditions and the following disclaimer in the documentation
	*			and/or other materials provided with the distribution.
	*	 3. Neither the name of STMicroelectronics nor the names of its contributors
	*			may be used to endorse or promote products derived from this software
	*			without specific prior written permission.
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
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
//#include "extern_declare.h"
#define ZEROPULSE 3777
#define SAMPLE_DELAY 2550
#define EIGHT_TWO_MSDELAY 8200
#define INPUT_RISE 'R'
#define INPUT_FALL 'F'
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
DAC_HandleTypeDef hdac;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t dbgflg = 0;
uint32_t dbg2 = 0;

int index_seq = 0;
int num = 0;

uint32_t IntegratorValue;
uint32_t ADCValue[1];

uint32_t OverSampleCounter = 0;
uint32_t SymbolCounter = 0;
uint32_t OnesCounter = 0;
uint32_t ZeroCounter = 0;
uint32_t ZeroDetected = 0;
uint32_t RUNONCEFLAG = 0;

uint32_t CurrentConvertedValue;
uint32_t PastConvertedValue;

uint32_t FallTimestamp;
uint32_t RiseTimestamp;
uint32_t TimeDiff;
uint8_t	EdgeFlag;
uint32_t FilteredSignal;
uint32_t EdgeDetectFlag;

uint32_t difference = 0;

uint32_t ValueCheck;
uint32_t PulseArray[80];
uint32_t PulseIndex = 0;
uint32_t CountEntries = 0;
uint32_t SymbolIndex = 0;
uint32_t RecoveredSignal = 0;
uint32_t CollectFlag = 0;

char buffer[10];
int n;

uint32_t SerialModeFlag = 0;
uint32_t SymbolReceived;
uint32_t SequenceFlags;
uint32_t SequenceIndex;
uint32_t StartFlag = 0;
uint8_t SerialSequenceReceived[8];
uint32_t SerialIndex = 8;
int start = 0;
int databegin = 0;
int arrayPos = 8;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
																

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void ADCBasicMode(void);
uint8_t EdgeDetect(uint32_t past_val, uint32_t curr_val, uint32_t* flag);
uint32_t PulseSymbolValidation(uint32_t rise_time, uint32_t fall_time);
uint32_t StartSequenceCheck(uint32_t last_symbol, uint32_t received_ptr, uint32_t index);
void OutputSymbol (void);
void SequenceCheck(void);
uint32_t IntegratorAdjust(uint32_t signal, uint32_t integrator);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	MX_DAC_Init();
	MX_TIM3_Init();
	MX_TIM6_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM1_Init();

	/* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, ADCValue, sizeof(uint16_t));
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

		/**Configure the main internal regulator output voltage 
		*/
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

		/**Initializes the CPU, AHB and APB busses clocks 
		*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 170;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

		/**Activate the Over-Drive mode 
		*/
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

		/**Initializes the CPU, AHB and APB busses clocks 
		*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
															|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

		/**Configure the Systick interrupt time 
		*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

		/**Configure the Systick 
		*/
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
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

		/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
		*/
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
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
		_Error_Handler(__FILE__, __LINE__);
	}

		/**DAC channel OUT1 config 
		*/
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 84;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0xffff;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 45;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 0xffff;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_IC_InitTypeDef sConfigIC;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 0xffff;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4024;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;

	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 0;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 56366;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
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
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** 
	* Enable DMA controller clock
	*/
static void MX_DMA_Init(void) 
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin|DecodedOutput_Pin|DebugOutput_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(FeedOut_GPIO_Port, FeedOut_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(TimerOut_GPIO_Port, TimerOut_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : FeedOut_Pin */
	GPIO_InitStruct.Pin = FeedOut_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(FeedOut_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DecodedOutput_Pin DebugOutput_Pin */
	GPIO_InitStruct.Pin = DecodedOutput_Pin|DebugOutput_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : TimerOut_Pin */
	GPIO_InitStruct.Pin = TimerOut_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TimerOut_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
	{
	if(htim->Instance == TIM4)
		{ 
		switch(EdgeFlag)
			{
			case INPUT_RISE:
				RiseTimestamp = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1);
				TIM_RESET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1);
				TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
				break;
			case INPUT_FALL:
				FallTimestamp = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1);
				TIM_RESET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1);
				TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
				FilteredSignal = PulseSymbolValidation(RiseTimestamp, FallTimestamp);
				//if(SerialModeFlag == 0){
					switch(FilteredSignal)
						{
						case 0:
						if (ZeroCounter > 40) //weirdness, just reset everything
						{
							start = 0;
							ZeroCounter = 0;
							OnesCounter = 0;
							arrayPos = 8;
						}
						if (start == 0)
						{
							HAL_GPIO_WritePin(DecodedOutput_GPIO_Port, DecodedOutput_Pin, GPIO_PIN_RESET);
						}
						else
						{
							//HAL_GPIO_WritePin(DecodedOutput_GPIO_Port, DecodedOutput_Pin, GPIO_PIN_SET);
						}
//							if (OnesCounter != 0)
//							{
//							HAL_GPIO_WritePin(DecodedOutput_GPIO_Port, DecodedOutput_Pin, GPIO_PIN_RESET);
//							}
//							if (start == 0)
//							{
//							//HAL_GPIO_WritePin(DecodedOutput_GPIO_Port, DecodedOutput_Pin, GPIO_PIN_RESET);
//							}
//							ZeroCounter++;
				if((OnesCounter <= 16) && (start == 0)) //this is a case where some noise might have gotten through
				{ 
						OnesCounter= 0;
////								//HAL_GPIO_WritePin(DecodedOutput_GPIO_Port, DecodedOutput_Pin, GPIO_PIN_RESET); //show normal/test zero
				}
				if((OnesCounter > 10) && (ZeroCounter > 4) && (start == 0)) //this is when the ones and zeros are both high enough to signify the start sequence
				{ 																									//counts up how many ONE pulses from matched filter and ZERO pulses after to determine start sequence
//								//if((htim5.Instance->CR1 && TIM_CR1_CEN) == 0)																								//altered to accept 1-0 start sequence
//										//HAL_TIM_Base_Start_IT(&htim5);
//										//RUNONCEFLAG++;
										start = 1;
										OnesCounter = 0; //keep the Ones count of the start sequence from messing with the first data bit
//										OnesCounter = 0;
//										ZeroCounter = 0;
//										//HAL_GPIO_WritePin(DecodedOutput_GPIO_Port, DecodedOutput_Pin, GPIO_PIN_SET);
				}
//							else if ((start == 1) && (databegin == 1)) //if data is being received -- this should be entered when the first return to zero happens.
//							{
//								if (arrayPos > 0) //data value received is between 1 and 8
//								{
	if (ZeroCounter == 0 && start == 1)
	{
				
				if (start == 1 && arrayPos > 1)
				{
								//HAL_GPIO_WritePin(DecodedOutput_GPIO_Port, DecodedOutput_Pin, GPIO_PIN_SET);
								if (OnesCounter > 26)
								{
									HAL_GPIO_WritePin(DecodedOutput_GPIO_Port, DecodedOutput_Pin, GPIO_PIN_RESET); //received 1-1-0, go low
									//SerialSequenceReceived[arrayPos - 1] = 0;
								}
								else
								{
									HAL_GPIO_WritePin(DecodedOutput_GPIO_Port, DecodedOutput_Pin, GPIO_PIN_SET); //received 1-0, go high
									//SerialSequenceReceived[arrayPos - 1] = 1;
								}
						OnesCounter = 0;
						ZeroCounter++;
						arrayPos--;
					}
					else if (start == 1 && arrayPos == 1)
					{
						if (OnesCounter > 24)
								{
									//HAL_GPIO_WritePin(DecodedOutput_GPIO_Port, DecodedOutput_Pin, GPIO_PIN_RESET); //received 1-1-0, go low
									//SerialSequenceReceived[arrayPos - 1] = 0;
								}
								else
								{
									//HAL_GPIO_WritePin(DecodedOutput_GPIO_Port, DecodedOutput_Pin, GPIO_PIN_SET); //received 1-0, go high
									//SerialSequenceReceived[arrayPos - 1] = 1;
								}
						arrayPos = 8;
						start = 0;
						OnesCounter = 0;
						ZeroCounter++;
					}
					else if (start == 0)
					{
						//HAL_GPIO_WritePin(DecodedOutput_GPIO_Port, DecodedOutput_Pin, GPIO_PIN_RESET);
						ZeroCounter++;
					}
					else
					{
						ZeroCounter++;
					}
	}
	else
	{
		ZeroCounter++;
	}
	
//								OnesCounter = 0;
//								ZeroCounter = 0;
//								arrayPos--; //prepare for next data bit
//								}
//								else //end of 8 bits of data
//								{
//									OutputSymbol(); //finished receiving 8 bits, output the data
//									databegin = 0; //reset begin and start to wait for start sequence again
//									start = 0;
//									arrayPos = 8; //reset array position to be ready for next set of data
//									OnesCounter = 0;
//									ZeroCounter = 0;
//									//HAL_GPIO_WritePin(DecodedOutput_GPIO_Port, DecodedOutput_Pin, GPIO_PIN_RESET);
//								}
//							}
							break;
	
	
						case 1:
							if (OnesCounter > 12 && start == 1)
							{
							//HAL_GPIO_WritePin(DecodedOutput_GPIO_Port, DecodedOutput_Pin, GPIO_PIN_SET);
							}
							else
							{
							//HAL_GPIO_WritePin(DecodedOutput_GPIO_Port, DecodedOutput_Pin, GPIO_PIN_RESET);
							}

//							if (start == 1 && databegin == 0) //this should be the first "1" after the start sequence is verified
//							{
//								databegin = 1; //show that data is starting to be received
//								OnesCounter = 0;	//reset to make sure of an accurate count
//								ZeroCounter = 0;
//							}
//							else
//							{
//							if (start == 0)
//							{
//								//HAL_GPIO_WritePin(DecodedOutput_GPIO_Port, DecodedOutput_Pin, GPIO_PIN_RESET); //show the high of the start sequence or a test case
//							}
//							else
//							{
//								//HAL_GPIO_WritePin(DecodedOutput_GPIO_Port, DecodedOutput_Pin, GPIO_PIN_RESET); //go low when getting a high data pulse
//							}
								OnesCounter++;
								ZeroCounter = 0;
//							}
						break;
//					}
				}
				break;
		}
	}
}
	
void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef* hadc){

	CurrentConvertedValue = ADCValue[0];
	
	EdgeFlag = EdgeDetect(PastConvertedValue, CurrentConvertedValue, &EdgeDetectFlag);	
	if(EdgeDetectFlag == 1){
		EdgeDetectFlag = 0;
		switch(EdgeFlag){
			case INPUT_RISE:
				HAL_GPIO_WritePin(FeedOut_GPIO_Port, FeedOut_Pin, GPIO_PIN_SET); //detected rising edge, flipped to RESET for inverter effect
				break;
			case INPUT_FALL:
				HAL_GPIO_WritePin(FeedOut_GPIO_Port, FeedOut_Pin, GPIO_PIN_RESET); //see above
				break;			
		}
	}
	PastConvertedValue = CurrentConvertedValue;	
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	SerialModeFlag = 1;
//	HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);
//	HAL_TIM_Base_Stop_IT(&htim6);
//	__HAL_TIM_SET_COUNTER(&htim4, 0);
//	__HAL_TIM_SET_COUNTER(&htim6, 0);
//	RiseTimestamp = 0;
//	FallTimestamp = 0;
//	TIM_RESET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1);
//	TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);	
//	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
}


//uncomment the period elapsed function for midpoint sampling

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	if(htim->Instance == TIM5){

//		if(OverSampleCounter < 15){
//			OverSampleCounter++;
//		}else{
//			SymbolCounter++;
//			OverSampleCounter = 0;
//		}
//		
//		if(OverSampleCounter == 14){	//CHANGE THIS TO CHANGE SAMPLING LOCATION
//			
//			// What does this do?
//			HAL_GPIO_TogglePin(DebugOutput_GPIO_Port, DebugOutput_Pin);

//			SerialSequenceReceived[SerialIndex-1] = FilteredSignal;
//			SerialIndex--;
//			if(SerialIndex == 0){ 
//				SerialIndex = 8;
//				StartFlag = 0;
//				OnesCounter = 0;
//				ZeroCounter = 0;
//				SymbolCounter = 0;
//				OverSampleCounter = 0;
//				
//				// What does this do?
//				while(HAL_TIM_Base_Stop_IT(&htim5) != HAL_OK);
//				
//				OutputSymbol();
//				
//				for(int i = 0; i<8; i++){
//					SerialSequenceReceived[i] = 0;
//				}
//				
//			}
//		}
//	}
//}



void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){

}

/* EdgeDetect
 * Looks at 2 sequential values to determine whether a valid 
 * edge transistion has occured.
 *
 * Outputs R or F to show whether edge was fall or rise,
 * and sets a 1 using a pointer to a flag.
 */
uint8_t EdgeDetect(uint32_t past_val, uint32_t curr_val, uint32_t* flag){
	uint8_t edge = 0;
	
	if((past_val >= MIDPOINT) && (curr_val <= MIDPOINT)){
		edge = INPUT_FALL; //falling edge
		*flag = 1;
	}else if((past_val <= MIDPOINT) && (curr_val >= MIDPOINT)){
		edge = INPUT_RISE; //Rising edge
		*flag = 1;
	}
	
	return edge;
}

/* PulseSymbolValidation
 * Calculates a pulse length, then determines whether the length was
 * within a valid threshold to be a valid transmission.
 *
 * Returns a flag to indicate a symbol was recieved, and writes a 1 
 * (only valid symbol pulses denote) to an address.
 */
uint32_t PulseSymbolValidation(uint32_t rise_time, uint32_t fall_time){
	uint32_t flag = 0;
	//____----_____
	if(rise_time < fall_time){ //difference calculation
		difference = fall_time -	rise_time;
	}else if(rise_time > fall_time){
		difference = ((0xffff - rise_time) + fall_time);
	}	
	if(difference <= ZEROPULSE){ //determining symbol
		flag = 1;
	}		
	return flag;
}


void OutputSymbol (void){
	/*	OutputSymbol
	 *	This function toggles an output pin to indicate whether a
   *	1 or 0 has been received.
   *	It also writes 1 or 0 to the UART (seen through USB COM Port
  */
	int p=0; 
	uint8_t buffer[100];
	uint8_t compressedsequence = 0;
	
//	p = sprintf((char *)buffer, "TESTING\r\n");
//	HAL_UART_Transmit(&huart2, buffer, p,50);

	for(int i = 0; i<8; i++){
		compressedsequence += SerialSequenceReceived[i] << (7-i);
	}
	if(compressedsequence == 13){	
		p = sprintf((char *)buffer, "%c", compressedsequence);
		dbg2 = HAL_UART_Transmit(&huart2, buffer, p,50);
	}else{
		p = sprintf((char *)buffer, "%c", compressedsequence);
		dbg2 = HAL_UART_Transmit(&huart2, buffer, p,50);
	}
	
	//HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
}

/* USER CODE END 4 */

/**
	* @brief	This function is executed in case of error occurrence.
	* @param	None
	* @retval None
	*/
void _Error_Handler(char * file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1) 
	{
	}
	/* USER CODE END Error_Handler_Debug */ 
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
