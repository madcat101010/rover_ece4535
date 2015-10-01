#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "queue.h"
#include "communication_public.h"

/*
typedef struct
{
	char seqNum;
	char type;
	int command;
	int duration;
} COMMUNICATION_MESSAGE;
//*/

typedef struct
{
	int seqNum;
	char msg;
} COMMUNICATION_MESSAGE;


typedef enum
{
	/* Application's state machine's initial state. */
	APP_STATE_INIT=0,
	APP_STATE_READ=1,
	APP_STATE_WRITE=2,
	APP_STATE_RECEIVE=3
} COMMUNICATION_STATES;


typedef struct
{
    COMMUNICATION_STATES state;
	QueueHandle_t theQueue;
	QueueHandle_t IntQueue;
	int TxMsgSeq;
	int RxMsgSeq;
	char WiflyTxMsgSeq;
	int IntTxMsgSeq;
	int IntRxMsgSeq;
	COMMUNICATION_MESSAGE rxMessage;
	COMMUNICATION_MESSAGE txMessage;
} COMMUNICATION_DATA;


void communication_UartTx(char* string);
void COMMUNICATION_Initialize ( void );
void communication_UartTxChar( char theChar );

void COMMUNICATION_Tasks( void );


#endif /* _APP_H */
