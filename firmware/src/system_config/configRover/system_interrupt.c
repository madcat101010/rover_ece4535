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

int i = 0;

	//25.6 us per count. USED FOR PWM OUTPUT
	//count to 200 = 51.2ms per rollover
void IntHandlerDrvTmrInstance0(void)

{
	//just reset upon rollover
	PLIB_TMR_Counter16BitClear(TMR_ID_2);
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
}

	//25.6 us per count. USED FOR PWM INPUT
	//count to 1000 = 256ms per rollover
IntHandlerDrvTmrInstance1(void)

{
	//just reset upon rollover
	PLIB_TMR_Counter16BitClear(TMR_ID_3);
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_3);
}
 
IntHandlerDrvTmrInstance2(void)

{
	motor_durationTick();
	PLIB_TMR_Counter16BitClear(TMR_ID_4);
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_4);
}

void IntHandlerDrvTmrInstance3(void)

{
	debugTimerTick();
	PLIB_TMR_Counter16BitClear(TMR_ID_5);	
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_5);
}
 
void IntHandlerDrvUsartInstance0(void)
{

    /* TODO: Add code to process interrupt here */

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
		while(!communication_IntQueueEmptyISR())
		{
			unsigned char txChar;
			txChar = communication_getByteISR();
			i++;
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
		PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);	//disable int due to empty xmit
	}
    /* Clear pending interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);

}
 

//sensors have 2 feedback lines... same output...
//left motor?
void IntHandlerDrvICInstance0(void)
{
//    PLIB_IC_FirstCaptureEdgeSelect(IC_ID_1, IC_EDGE_FALLING);
//	motor_sendmsgISR(11, );
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_1);
}

//right motor?
void IntHandlerDrvICInstance1(void)
{
//	motor_sendmsgISR(12, );
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_2);
}

 
/*******************************************************************************
 End of File
*/

