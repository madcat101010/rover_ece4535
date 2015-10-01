
#include "communication.h"

#define COMMUNICATIONQUEUESIZE 15
COMMUNICATION_DATA communicationData;


// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************


void communication_sendmsg(int command, int duration)
{
	COMMUNICATION_MESSAGE theMessage;
	theMessage.command = command;
	theMessage.duration = duration;
	theMessage.type = 0x00;
	if(communicationData.theQueue != 0)
	{
		if(xQueueSend(communicationData.theQueue, (void*)&(theMessage), portMAX_DELAY) == pdTRUE)
		{
			communicationData.TxMsgSeq++;
		}
	}
}
void communication_sendmsgISR(int command, int duration)
{
	COMMUNICATION_MESSAGE theMessage;
	theMessage.command = command;
	theMessage.duration = duration;
	theMessage.type = 0x00;
	if(communicationData.theQueue != 0)
	{
		if(xQueueSendFromISR(communicationData.theQueue, (void*)&(theMessage), 0) == pdTRUE)
		{
			communicationData.TxMsgSeq++;
		}
	}
}

void communication_sendIntMsg(int left, int right)
{
	COMMUNICATION_MESSAGE theMessage;
	//send message sequence byte
	theMessage.seqNum = communicationData.IntTxMsgSeq;
	theMessage.msg = communicationData.WiflyTxMsgSeq;
	if(xQueueSend(communicationData.theQueue, (void*)&(theMessage), portMAX_DELAY) == pdTRUE)
		communicationData.IntTxMsgSeq++;
	communicationData.WiflyTxMsgSeq++;
	//send from rover byte
	theMessage.seqNum = communicationData.IntTxMsgSeq;
	theMessage.msg = 0x00;	
	if(xQueueSend(communicationData.theQueue, (void*)&(theMessage), portMAX_DELAY) == pdTRUE)
		communicationData.IntTxMsgSeq++;
	//send left 1 of 4
	theMessage.seqNum = communicationData.IntTxMsgSeq;
	theMessage.msg = (char)left;	
	if(xQueueSend(communicationData.theQueue, (void*)&(theMessage), portMAX_DELAY) == pdTRUE)
		communicationData.IntTxMsgSeq++;
	//send left 2 of 4
	theMessage.seqNum = communicationData.IntTxMsgSeq;
	theMessage.msg = (char)(left >> 8);	
	if(xQueueSend(communicationData.theQueue, (void*)&(theMessage), portMAX_DELAY) == pdTRUE)
		communicationData.IntTxMsgSeq++;
	//send left 3 of 4
	theMessage.seqNum = communicationData.IntTxMsgSeq;
	theMessage.msg = (char)(left >> 16);	
	if(xQueueSend(communicationData.theQueue, (void*)&(theMessage), portMAX_DELAY) == pdTRUE)
		communicationData.IntTxMsgSeq++;
	//send 4 of 4
	theMessage.seqNum = communicationData.IntTxMsgSeq;
	theMessage.msg = (char)(left >> 24);	
	if(xQueueSend(communicationData.theQueue, (void*)&(theMessage), portMAX_DELAY) == pdTRUE)
		communicationData.IntTxMsgSeq++;
	//send right 1 of 4
	theMessage.seqNum = communicationData.IntTxMsgSeq;
	theMessage.msg = (char)right;	
	if(xQueueSend(communicationData.theQueue, (void*)&(theMessage), portMAX_DELAY) == pdTRUE)
		communicationData.IntTxMsgSeq++;
	//send right 2 of 4
	theMessage.seqNum = communicationData.IntTxMsgSeq;
	theMessage.msg = (char)(right >> 8);	
	if(xQueueSend(communicationData.theQueue, (void*)&(theMessage), portMAX_DELAY) == pdTRUE)
		communicationData.IntTxMsgSeq++;
	//send right 3 of 4
	theMessage.seqNum = communicationData.IntTxMsgSeq;
	theMessage.msg = (char)(right >> 16);	
	if(xQueueSend(communicationData.theQueue, (void*)&(theMessage), portMAX_DELAY) == pdTRUE)
		communicationData.IntTxMsgSeq++;
	//send right 4 of 4
	theMessage.seqNum = communicationData.IntTxMsgSeq;
	theMessage.msg = (char)(right >> 24);	
	if(xQueueSend(communicationData.theQueue, (void*)&(theMessage), portMAX_DELAY) == pdTRUE)
		communicationData.IntTxMsgSeq++;
}

char communication_getByteISR()
{
	COMMUNICATION_MESSAGE theMessage;
	if(xQueueReceiveFromISR(communicationData.IntQueue, (void*)&(theMessage), 0))
	{
		int i = 0;
		while( theMessage.seqNum != communicationData.IntRxMsgSeq)
		{
			i++;																		
			if(i == COMMUNICATIONQUEUESIZE)
			{
				communicationData.IntRxMsgSeq++;		//drop a message of that seq number by incrementing
				i = 0;
			}
			xQueueSendToBackFromISR(communicationData.theQueue, (void*)&(theMessage), 0);	//send to back
			xQueueReceiveFromISR(communicationData.theQueue, (void*)&(theMessage), 0 );	//get another
		}
		return theMessage.msg;
	}
}

bool communication_IntQueueEmptyISR()
{
	BaseType_t retval;
	retval = xQueueIsQueueEmptyFromISR(communicationData.IntQueue);
	if(retval == pdTRUE)
		return true;
	else
		return false;
}

void communication_UartTx( char* string )
{
	int i = 0;
	char getChar = ' ';
	while(i < string[i] != 0)
	{
		getChar = string[i];
		DRV_USART1_WriteByte(getChar);
		i++;
	}		
}

////####################
//put transmit in interrupt handler. interrupt when queue has room/can send and then pull characters
//the interrupt handly checks its own queue so tha tother threads can use it. 
//make a seperate transmit app file for this... not in system_interrupt.c
//when i want to send a message, enable the interrupt. turn off interrupt when there are no messages in thequeue.
//try to avoid blocks as much as possible for ISRs. also prevents blockings on xmit
void communication_UartTxChar( char theChar )
{
	DRV_USART1_WriteByte(theChar);
}

// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
void COMMUNICATION_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
	initDebugU();
    communicationData.state = APP_STATE_INIT;
	communicationData.rxMessage.type = 0;
	communicationData.txMessage.type = 0;
	communicationData.theQueue = xQueueCreate(COMMUNICATIONQUEUESIZE, sizeof(COMMUNICATION_MESSAGE)); //sizeof(communicationData.rxMessage));
	if(communicationData.theQueue == 0)
	{
		crash("E: Comm msgQ");
	}
	communicationData.IntQueue = xQueueCreate(COMMUNICATIONQUEUESIZE, sizeof(COMMUNICATION_MESSAGE)); //sizeof(communicationData.rxMessage));
	if(communicationData.IntQueue == 0)
	{
		crash("E: Int msgQ");
	}
	DRV_USART1_Initialize();
}


void COMMUNICATION_Tasks ( void )
{
	while(1)
	{
		//check if queue exists
		if(communicationData.theQueue != 0)	
		{
			//receive a message and store into rxMessage ... block 5 ticks if empty queue
			if(xQueueReceive(communicationData.theQueue, &(communicationData.rxMessage), portMAX_DELAY ))
			{
				int i = 0;
				while( communicationData.rxMessage.seqNum != communicationData.RxMsgSeq)
				{
					i++;																		
					if(i == COMMUNICATIONQUEUESIZE)
					{
						communicationData.RxMsgSeq++;		//drop a message of that seq number by incrementing
						i = 0;
					}
					xQueueSendToBack(communicationData.theQueue, (void*)&(communicationData.rxMessage), 0);					//send to back
					xQueueReceive(communicationData.theQueue, (void*)&(communicationData.rxMessage), 0 );	//get another
				}
				//received messageQ message will tell use the state which will be used
				//##communicationData.state = communicationData.rxMessage.type;
				communicationData.state = 2;
				
				//run the state according to the message's state
				switch ( communicationData.state )
				{
					/* Application's initial state. */
					case APP_STATE_INIT:	//0
						break;

					case APP_STATE_READ:	//1
					{
						debugU("Read");
						communication_UartRx();
						communicationData.state = 0;
						communication_sendmsg(2,communicationData.rxBuffer);
						communicationData.rxMessage.type = 0;
						communicationData.rxMessage.string = "";
						break;
					}

					case APP_STATE_WRITE:	//2
					{
						debugU("Write");
						communication_UartTxChar(communicationData.rxChar);
						communicationData.state = 0;
						communicationData.rxMessage.type = 0;
						communicationData.rxMessage.string = "";
						//communicationData.rxChar = ' ';
						break;
					}
					case APP_STATE_RECEIVE:	//3
					{
						debugU("Receiving");
						if(communicationData.rxBufferIndex < communicationData.rxBufferSize)	//if we are within buffer bounds, copy char
							communicationData.rxBuffer[communicationData.rxBufferIndex] = communicationData.rxMessage.string[0];
						communicationData.rxBufferIndex++;
						communicationData.state = 0;
						communicationData.rxMessage.type = 0;
						communicationData.rxMessage.string = "";
						break;
					}

					/* The default state should never be executed. */
					default:
					{
						/* TODO: Handle error in application's state machine. */
						break;
					}
				}//end of case
			}//end of receive message
		}//end of check message queue != 0
		else	//attempt to create the queue again if it doesn't exist
		{
			//something terrible happens normally so have it exit rather than recreate
			crash("E: No Comm Q");
			//run exit interrupt to OS... or forever loop message
			return;
			//communicationData.theQueue = xQueueCreate(10, sizeof(APP_STATES));
		}
	}//end of while(1)
}
 
