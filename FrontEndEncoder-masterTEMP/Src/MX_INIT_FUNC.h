void SystemClock_Config(void);
void Error_Handler(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_DAC_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_USART3_UART_Init(void);
void MX_TIM4_Init(void);
void MX_USART2_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
