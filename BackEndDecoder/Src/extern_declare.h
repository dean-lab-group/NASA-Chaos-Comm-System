#include "stm32f4xx_hal.h"

extern uint32_t IntegratorValue;
extern uint32_t ADCValue[1];

extern uint32_t CurrentConvertedValue;
extern uint32_t PastConvertedValue;

extern uint32_t FallTimestamp;
extern uint32_t RiseTimestamp;
extern uint32_t TimeDiff;

extern uint8_t  EdgeFlag;
extern uint32_t ZeroFlag;
extern uint32_t ReceiveFlag;
extern uint32_t AlreadyProcessedFlag;
extern uint32_t EdgeDetectFlag;
extern uint32_t SymbolRecep;

extern char buffer[10];
extern int n;

extern volatile uint32_t CustomDataMode;
extern volatile uint32_t CustomData;
extern volatile uint32_t CustomIndex;
extern volatile uint32_t CustomModeEdgeFlag;

extern uint32_t SymbolReceived;
