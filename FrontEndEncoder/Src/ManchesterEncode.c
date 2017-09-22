#include "stm32f4xx_hal.h"
#include "ManchesterEncode.h"

extern volatile int togglecheck;


void ManchesterOne(void){
int j = 0;
	
	for(j=0;j<ONES_REPEATED;j++){
		while(togglecheck == 0){
		}
		HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_SET);	
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		togglecheck = 0;
	}	
	//HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_RESET);	
	
//	for(j=0;j<6;j++){
//		while(togglecheck == 0){
//		}
//		HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//		togglecheck = 0;
//	}
//	HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_SET);
//	
}

void ManchesterZero(void){
	int j = 0;
	
	for(j=0;j<ZEROES_REPEATED;j++){
		while(togglecheck == 0){
		}
		HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		togglecheck = 0;
	}
	//HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_SET);
	
//	for(j=0;j<10;j++){
//		while(togglecheck == 0){
//		}
//		HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_SET);	
//		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//		togglecheck = 0;
//	}
//	HAL_GPIO_WritePin(TransistorSwitch_GPIO_Port, TransistorSwitch_Pin, GPIO_PIN_RESET);	
	
}
