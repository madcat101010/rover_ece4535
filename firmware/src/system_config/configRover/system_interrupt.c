/*
#ifndef r_DEBUG
#define r_DEBUG 1
#endif
//*/

/*******************************************************************************
 System Interrupts File

  File Name:
    system_int.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <xc.h>
#include <sys/attribs.h>
#include "communication.h"
#include "motor.h"
#include "system_definitions.h"
#include "system_config.h"
// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

int LedgeType = 0;
int RedgeType = 0;
int i = 0;
float Ravg = 0;
float Lavg = 0;
int temp = 0;
int LTotal = 0;
int RTotal = 0;
float n = 0;

//right encoder pin 1
void IntHandlerExternalInterruptInstance0(void)
{   
//	LedgeType = (LedgeType+1)%2;
//	SYS_INT_ExternalInterruptTriggerSet(INT_EXTERNAL_INT_SOURCE1,LedgeType);
	motor_REncode();
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_1);
}

//left encoder pin 7
void IntHandlerExternalInterruptInstance1(void)
{           
//	RedgeType = (RedgeType+1)%2;
//	SYS_INT_ExternalInterruptTriggerSet(INT_EXTERNAL_INT_SOURCE2,RedgeType);
	motor_LEncode();
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_2);
}

//PWM Output timer of 10417Hz
void IntHandlerDrvTmrInstance0(void)
{
	//just reset upon rollover
	PLIB_TMR_Counter16BitClear(TMR_ID_2);
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
}

//5ms timer
IntHandlerDrvTmrInstance1(void)
{
	//just reset upon rollover
/*	n = n+1;
	temp = motor_ResetLEncode();
	LTotal += temp;
	debugUInt(temp);
	debugUInt(LTotal);
	Lavg = ( (float)temp + (Lavg*(n-1)) ) / n;
	debugUFloat(Lavg);
//*/
/*	
	temp = motor_ResetREncode();
	RTotal += temp;
	debugUInt(temp);
	debugUInt(RTotal);
	Ravg = ( (float)temp + (Ravg*(n-1)) ) / n;
	debugUFloat(Ravg);
//*/	
	//RUN PID HERE?
	
	PLIB_TMR_Counter16BitClear(TMR_ID_3);
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_3);
}
 
//1 ms timer.
void IntHandlerDrvTmrInstance2(void)
{
	motor_durationTick();
	debugTimerTick();
	PLIB_TMR_Counter16BitClear(TMR_ID_5);	
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_5);
}

//5Hz timer
void IntHandlerDrvTmrInstance3(void)
{
	communication_sendIntMsgFromISR(motor_ResetLEncodeTotal(), motor_ResetREncodeTotal());
	PLIB_TMR_Counter16BitClear(TMR_ID_4);	
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_4);
}

void IntHandlerDrvUsartInstance0(void)
{
	if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT))
	{
		//check if queue is not empty first...
		//USART_TRANSMIT_FIFO_EMPTY
		int pop = 0;
		while(pop < 6 && !debugUQueueEmptyISR())
		{
			unsigned char txChar;
			txChar = debugUGetByteISR();
			DRV_USART0_WriteByte(txChar);
			pop++;
		}
		if(debugUQueueEmptyISR())
			PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT);	//disable int due to empty xmit
	}

    /* Clear pending interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_RECEIVE);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_ERROR);

}
 
 


void IntHandlerDrvUsartInstance1(void)
{

    /* TODO: Add code to process interrupt here */
	if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE))
	{
		while(!DRV_USART1_ReceiverBufferIsEmpty()) //grab everyhting in the buffer
		{
			unsigned char msg = DRV_USART1_ReadByte(); // read received byte
			communication_sendmsgISR(msg,1);
		}
	}
	if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT))
	{
		//check if queue is not empty first...
		int pop = 0;
		while(!communication_IntQueueEmptyISR() && pop < 6)
		{
			unsigned char txChar;
			txChar = communication_getByteISR();
			pop++;
	//		if(i% 20 != 1)
			DRV_USART1_WriteByte(txChar);
#ifdef r_DEBUG
			debugU("COM tx: ");
			debugUInt(txChar);
#endif
		}
//		else
//		{
//			PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);	//disable int due to empty xmit
//		}
		if(communication_IntQueueEmptyISR())
			PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);	//disable int due to empty xmit
	}
    /* Clear pending interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);

}
 



 
 

 
 
 
 
/*******************************************************************************
 End of File
*/

