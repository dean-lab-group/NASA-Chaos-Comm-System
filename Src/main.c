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
#include "ManchesterEncode.h"
#include "MX_INIT_FUNC.h"

/* USER CODE BEGIN Includes */
#define OFFSET 30
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static int TestArray[48] = {1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0};
	//To prevent transients on the oscillator, multiple copies of the same symbol must be sent in a row.
	//This sequency is actually 1010110010
int ArrLen = sizeof(TestArray) / sizeof(int);
int ArrInd = 0;
int NextSymbol = 85, CurrentSymbol = 85;
	//85 = invalid input, prevents accidentaly entering loops

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


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

                

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


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//		while(!StartFlag){
//		}
		//StartFlag = 0;
		switch(CaseNumber){
			case(0) :
				while(togglecheck){ //if it's time to switch to next symbol
					TestCaseOutput(); //transmit
				}
				break;
			case(1) :
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

/* USER CODE BEGIN 4 */
void TestCaseOutput(void){
	
	if(NextSymbol == 85){ //data validation for the initial transmission
		NextSymbol = TestArray[ArrInd];  //load the first transmission symbol in the array
		ArrInd++;
	}
	
	if(NextSymbol == 1){
		HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_SET);	
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	}else if(NextSymbol == 0){
		HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_RESET);
	}
	
	if((ArrInd + 1) <= (ArrLen)){ //prevent ArrInd going out of bounds
		CurrentSymbol = NextSymbol;
		NextSymbol = TestArray[ArrInd];
		ArrInd++;
		if(ArrInd == ArrLen){
			ArrInd = 0;		
		}
	}
	togglecheck = 0; //clear flag
}

void ReadCustomData(void){
	static int p=0; //,i = 0;
	uint8_t buffer[100];
	uint8_t recbuff[1];
	uint8_t sendbuff[8];
	
	p = sprintf((char *)buffer, "Please enter in your own input:\n");
	HAL_UART_Transmit(&huart2, buffer, p,50);
	HAL_UART_Receive(&huart2, recbuff, 1, 10000);
	p = 0;
	for(i=0;i<8;i++){
		sendbuff[i] = (recbuff[0] & (0x1 << i));
		sendbuff[i] = sendbuff[i] >> i; 
		//p++;
	}
			
	for(int j=0;j<12;j++){ //Flag a start to the sequence
				while(!togglecheck){
				}
				HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_SET);	
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
				togglecheck = 0;
			}
	HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_RESET);
				
	for(i=0;i<8;i++){
		if(sendbuff[i] == 1){
			ManchesterOne();
		}else if(sendbuff[i] == 0){
			ManchesterZero();			
		}
	}
	
	while(!togglecheck){ //set back to low
	}
	HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	togglecheck = 0;

	
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) //IGNORE THIS CALLBACK --UNUSED
{
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); //pin toggle to ensure operation
		
	if(LastValue <= 4095){
		if((LastValue < LOWEDGE) && (ADCConvertedValue > HIGHEDGE)){
			EdgeCount++;
			if(HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
				{
					Error_Handler();
				}
		}
		
//		if(CurrentSymbol == 1){
//			if(EdgeCount >= HIGHCOUNT){
//				EdgeCount = 0;
//				togglecheck = 1;
//			}
//		}else if(CurrentSymbol == 0){
//			if(EdgeCount >= LOWCOUNT){
//				EdgeCount = 0;
//				togglecheck = 1;
//			}
//		}
	}

	LastValue = ADCConvertedValue; //store last state
}

void HAL_TIM_IC_CaptureCallback( TIM_HandleTypeDef* htim ){
	
	
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
