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