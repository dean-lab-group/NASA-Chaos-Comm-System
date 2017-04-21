/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "MX_INIT_FUNC.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t ADCConvArr[1];
uint32_t CurrentConvertedValue = 0;
uint32_t PastConvertedValue = 0;

uint32_t FallTimestamp = 0;
uint32_t RiseTimestamp = 0;
uint32_t TimeDiff = 0;

uint8_t  EdgeFlag = 0;
uint32_t ZeroFlag = 0;
uint32_t ReceiveFlag = 0;
uint32_t AlreadyProcessedFlag = 0;
uint32_t EdgeDetectFlag = 0;
uint32_t SymbolRecep = 0;

char buffer[10];
int n = 0;

volatile uint32_t CustomDataMode = 0;
volatile uint32_t CustomData = 0;
volatile uint32_t CustomIndex = 0;
volatile uint32_t CustomModeEdgeFlag = 0;

uint32_t SymbolReceived = 86;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void OutputSymbol(void);
uint8_t EdgeDetect(uint32_t past_val, uint32_t curr_val, uint32_t* flag);
uint32_t PulseSymbolValidation(uint32_t rise_time, uint32_t fall_time, uint32_t* symbol);
void ADCBasicMode(void);
void ADCCustomData(void);
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

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_TIM8_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2); //time base used for ADC sample rate
	HAL_TIM_Base_Start(&htim3); //time base used for timing detected edge intervals
	while(HAL_ADC_Start_DMA(&hadc1, ADCConvArr, 1) != HAL_OK){ //starting ADC
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		while(ReceiveFlag){
			ReceiveFlag = 0;
			OutputSymbol();
		}
  }
  /* USER CODE END 3 */

}

/* USER CODE BEGIN 4 */

/*  OutputSymbol
 *  This function toggles an output pin to indicate whether a
 *  1 or 0 has been received.
 *  It also writes 1 or 0 to the UART (seen through USB COM Port
 */
void OutputSymbol (void){
	//This function is used to actually indicate what symbol has been detected/
	// decoded, by writing an output pin to the corresponding state as well as
	// writing the value over UART
	
	if(ZeroFlag && !AlreadyProcessedFlag){	//This is if a zero was detected, but not processed yet
		n = sprintf(buffer, "%d\n\r", 0);
		HAL_UART_Transmit_IT(&huart2, (uint8_t*) buffer, n);
		HAL_GPIO_WritePin(FilteredOutput_GPIO_Port, FilteredOutput_Pin, GPIO_PIN_RESET);	
		AlreadyProcessedFlag = 1; //show zero has been processed, and clear the zero flag
		ZeroFlag = 0;	
	}else if(!ZeroFlag){ //if no zero was detected by the timer
		n = sprintf(buffer, "%d\n\r", 1);
		HAL_UART_Transmit_IT(&huart2, (uint8_t*) buffer, n);
		HAL_GPIO_WritePin(FilteredOutput_GPIO_Port, FilteredOutput_Pin, GPIO_PIN_SET);	
	}
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
	
	if((past_val <= LOWEDGE) && (curr_val >= HIGHEDGE)){
		edge = 'R'; //Rising edge
		*flag = 1; //edge has been detected
	}else if((past_val >= HIGHEDGE) && (curr_val <= LOWEDGE)){
		edge = 'F'; //falling edge
		*flag = 1; //edge has been detected
	}
	return edge;
}

/* PulseSymbolValidation
 * Calculates a pulse length, then determines whether the length was
 * within a valid threshold to be a valid transmission.
 *
 * Returns a flag to indicate a symbol was recieved, and writes a 1 
 * (only valid symbol pulses dnote) to an address.
 */
uint32_t PulseSymbolValidation(uint32_t rise_time, uint32_t fall_time, uint32_t* symbol){
	uint32_t difference = 0;
	uint32_t flag = 0;
	
	if(rise_time > fall_time){ //difference calculation
		difference = rise_time - fall_time;
	}else if(rise_time < fall_time){
		difference = ((0xFFFF - fall_time) + rise_time);
	}
	if(difference <= ZEROPULSE){ //determining symbol
		*symbol = 1;
		flag = 1;
		}				///take RiseTime, FallTIme, symboladdr,::::return receiveflag
	return flag;
}

/* ADCBasicMode
 * Contains edge detection and processing logic for
 * "debug" mode.
 */
void ADCBasicMode(void){
	//Save the previously converted value and pull the current converted value
	PastConvertedValue = CurrentConvertedValue;
	CurrentConvertedValue = ADCConvArr[0];
	EdgeFlag = EdgeDetect(PastConvertedValue, CurrentConvertedValue, &EdgeDetectFlag);
	
	switch(EdgeFlag){
		case 0:
			break;
		case 'F':
				RiseTimestamp = __HAL_TIM_GetCounter(&htim3); // pull the current counter value
			break;
		case 'R':
				FallTimestamp = __HAL_TIM_GET_COUNTER(&htim3); //get time stamp
			break;
	}
	
	if(EdgeFlag != 0){ 
		if(ZeroFlag){ 
			SymbolReceived = 0; 
			ReceiveFlag = 1; 
		}	
		if(EdgeFlag == 'F' && EdgeDetectFlag && !ZeroFlag){
			EdgeDetectFlag = 0; 
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, __HAL_TIM_GET_COUNTER(&htim3) + ZEROPULSE);
			HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
			
		}else if(EdgeFlag == 'R' && EdgeDetectFlag && !AlreadyProcessedFlag){	
			if(!ZeroFlag){
				HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
				ReceiveFlag = PulseSymbolValidation(RiseTimestamp, FallTimestamp, &SymbolReceived);
			}
		}else if(AlreadyProcessedFlag){
			AlreadyProcessedFlag = 0;
		}
	}
}

void ADCCustomData(void){
	
}

void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef* hadc){
			
	// This callback is called every time the ADC finishes a conversion
	// It contains the edge detection & symbol decoding logic
	switch (CustomDataMode){	
		case 0:
			ADCBasicMode();
			break;
		case 1:
			ADCCustomData();
			break;		
	}
	
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(!CustomDataMode){
		CustomDataMode++;
	}
}


void HAL_TIM_OC_DelayElapsedCallback( TIM_HandleTypeDef* htim){
	
	//a zero is encoded as a constant low pulse for the length of the symbol,
	//which is decoded by starting a timer. When the length of the symbol has elapsed
	// this callback flags a zero
	if(htim == &htim2){
		//htim2 is just for ADC sampling rate; do nothing
	}else if(htim == &htim3){ //long enough pulse to be a zero
		ZeroFlag = 1;	
		HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
		HAL_TIM_Base_Start(&htim3);
	}
}
/* USER CODE END 4 */



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
