#include "debug.h"


int lastTime;
int msTime;
QueueHandle_t debugQueue;

///*
void debugCharInit()
{
	PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_E, 0x00FF);
}

//output character
void debugChar(int PinByteSel)
{
	PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, (PinByteSel & 0x00FF));
}
//*/

//USART0 Driver is connected to USART ID 2
void initDebugU()
{
	debugQueue = xQueueCreate(200, sizeof(char)); //sizeof(communicationData.rxMessage));
	if(debugQueue == 0)
	{
		crash("E: Comm msgQ");
	}
		DRV_USART0_Initialize();
}

void debugU(char* debugMessage)
{
	int i = 0;
	char getChar = ' ';
	while(i < debugMessage[i] != 0)
	{
		xQueueSendFromISR(debugQueue, (void*)&(debugMessage[i]), 0);
		/*
		getChar = debugMessage[i];
		DRV_USART0_WriteByte(getChar);
		//*/
		i++;
	}	
	PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT);	//ENABLE TX INTERRUPT
//	DRV_USART0_WriteByte('\r');
}

bool debugUQueueEmptyISR()
{
	BaseType_t retval;
	retval = xQueueIsQueueEmptyFromISR(debugQueue);
	if(retval == pdTRUE)
		return true;
	else
		return false;
}

unsigned char debugUGetByteISR()
{
	char theMessage;
	if(xQueueReceiveFromISR(debugQueue, (void*)&(theMessage), 0))
	{
			return theMessage;
	}
	return 0;
}

void debugUChar(char theChar)
{
//	debugU(&theChar);
	debugU("\r");
//	DRV_USART0_WriteByte(theChar);
//	DRV_USART0_WriteByte('\r');	
}

void debugUInt(int number)
{
	char str[12];
	sprintf(str, "%d", number);
	debugU(str);
	debugU("\r");
//	DRV_USART0_WriteByte('\r');
}

void debugUFloat(float number)
{
	char str[25];
	snprintf(str, 25, "%f", number);
	debugU(str);
	debugU("\r");
//	DRV_USART0_WriteByte('\r');
}


void crash(char* debugMessage)
{
	debugU("CRASHED: ");
	debugU(debugMessage);
}


void debugTimerInit()
{
//	DRV_TMR3_Initialize();	
//	DRV_TMR3_CounterClear();
//	DRV_TMR3_Start();
	lastTime = 0;
	msTime = 0;
}

void debugTimerTick()
{
	msTime++;
}


int debugGetTime()
{
	lastTime = msTime;
	msTime = 0;
	return lastTime;
}