
#include "motor.h"
#include "peripheral/oc/plib_oc.h"

#define MOTORQUEUELENGTH 10
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

MOTOR_DATA motorData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/
void motor_sendmsg(int command, int duration)
{
	MOTOR_MESSAGE theMessage;
	theMessage.SeqNum = 0;//motorData.TxSeqNum;
	theMessage.command = command;
	theMessage.duration = duration;
	if(motorData.theQueue != 0)
	{
		if( xQueueSend(motorData.theQueue, (void*)&(theMessage), portMAX_DELAY))
		{
			motorData.TxSeqNum++;
		}
	}
}

void motor_sendmsgISR(int command, int duration)
{
	MOTOR_MESSAGE theMessage;
	theMessage.SeqNum = motorData.TxSeqNum;
	theMessage.command = command;
	theMessage.duration = duration;
	if(motorData.theQueue != 0)
	{
		if( xQueueSendFromISR(motorData.theQueue, (void*)&(theMessage), 0) == pdTRUE)
		{
			motorData.TxSeqNum++;
		}
	}
}


void motor_durationTick()
{
	if( motorData.state != 0)
	{
		if( motorData.duration == 0)
		{
			motor_sendmsgISR(0, 0);
			
		}
		else
			motorData.duration--;
	}
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MOTOR_Initialize ( void )

  Remarks:
    See prototype in motor.h.
 */

void MOTOR_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    motorData.state = MOTOR_STATE_INIT;
        /* Place the App state machine in its initial state. */
	motorData.RxSeqNum = 0;
	motorData.TxSeqNum = 0;
	//motorData.rxMessage.command = 0;
	//motorData.rxMessage.duration = 0;
	motorData.theQueue = xQueueCreate(MOTORQUEUELENGTH, sizeof(MOTOR_MESSAGE)); //sizeof(appData.rxMessage));
	if(motorData.theQueue == 0)
	{
		crash("E:Create Motor msgQ");
		//failed to create queue
	}
	PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_G, 0x0002);
	PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_D, 0x0003);
	PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_C, 0x4000);
	PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_1, 0);
	PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_0, 0);

	DRV_OC1_Initialize();
	DRV_OC0_Initialize();
	//DRV_OC1_Enable();
	//DRV_OC0_Enable();	//same thing as start
	DRV_OC1_Stop();
	DRV_OC0_Stop();
	//25.6 us per count
	//count to 200 = 51.2ms per rollover
	DRV_TMR0_Initialize();	//OC Timer
	DRV_TMR1_Initialize();	//IC Timer
	DRV_TMR2_Initialize();	//duration Timer
	DRV_TMR0_CounterClear();
	DRV_TMR1_CounterClear();
	DRV_TMR2_CounterClear();
	DRV_TMR0_Start();
	DRV_TMR1_Stop();
	DRV_TMR2_Stop();
	
	initDebugU();
//	theTimerInit(52);
}


/******************************************************************************
  Function:
    void MOTOR_Tasks ( void )

  Remarks:
    See prototype in motor.h.
 */

void MOTOR_Tasks ( void )
{
	while(1)
	{
		if(motorData.theQueue != 0)
		{
			if(xQueuePeek(motorData.theQueue, (void*)&(motorData.rxMessage), portMAX_DELAY ))
			{
				xQueueReceive(motorData.theQueue, (void*)&(motorData.rxMessage), portMAX_DELAY );

				/*			int i = 0;
				while( motorData.rxMessage.SeqNum != motorData.RxSeqNum)
				{
					i++;																		
					if(i == MOTORQUEUELENGTH)
					{
						motorData.RxSeqNum++;		//drop a message of that seq number by incrementing
						i = 0;
					}
					xQueueSendToBack(motorData.theQueue, (void*)&(motorData.rxMessage), 0);					//send to back
					xQueueReceive(motorData.theQueue, (void*)&(motorData.rxMessage), 0 );	//get another
				}//*/
				motorData.state = motorData.rxMessage.command;
//				motorData.state = 1;
//				debugUInt(motorData.state);
//				debugUInt(motorData.duration);
				communication_sendIntMsg(motorData.rxMessage.command, motorData.rxMessage.duration);
				/* Check the application's current state. */
				switch ( motorData.state )
				{
					/* Application's initial state. */
					case MOTOR_STATE_INIT:	//state 0, do nothing
					{
						DRV_TMR1_Stop();
						DRV_TMR2_Stop();
						DRV_TMR1_CounterClear();
						DRV_TMR2_CounterClear();
						DRV_OC1_Stop();
						DRV_OC0_Stop();
						motorData.duration = motorData.rxMessage.duration;
						break;
					}
					case MOTOR_STATE_STRAIGHT: //state 1, go straight.
					{
						//TMRCount < OCSetting = high... 1200/1600 = 75% duty cycle
//						PLIB_OC_PulseWidth16BitSet(OC_ID_1, 1200);
//						PLIB_OC_PulseWidth16BitSet(OC_ID_2, 1200);
						PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1, 0);
						PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14, 0);
						DRV_OC1_Start();
						DRV_OC0_Start();
						motorData.duration = motorData.rxMessage.duration;
						DRV_TMR1_Start();
						DRV_TMR2_Start();
						break;
					}
					case MOTOR_STATE_TURNLEFT: //state 2, turn left.
					{
						PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1, 1);
						PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14, 0);
						DRV_OC1_Start();
						DRV_OC0_Start();
						motorData.duration = motorData.rxMessage.duration;
						DRV_TMR1_Start();
						DRV_TMR2_Start();
						break;
					}
					case MOTOR_STATE_TURNRIGHT: //state 3, turn right
					{
						PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1, 0);
						PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14, 1);
						DRV_OC1_Start();
						DRV_OC0_Start();
						motorData.duration = motorData.rxMessage.duration;
						DRV_TMR1_Start();
						DRV_TMR2_Start();
						break;
					}
					case MOTOR_STATE_LEFTFEEDBACK:
					{
						motorData.Lfeedback = motorData.rxMessage.duration;
						break;
					}
					case MOTOR_STATE_RIGHTFEEDBACK:
					{
						motorData.Rfeedback = motorData.rxMessage.duration;
						break;
					}
					/* TODO: implement your application state machine.*/

					case MOTOR_STATE_Straight2: //state 1, go straight.
					{
						//TMRCount < OCSetting = high... 150/200 = 75% duty cycle
//						PLIB_OC_PulseWidth16BitSet(OC_ID_1, 150);
//						PLIB_OC_PulseWidth16BitSet(OC_ID_2, 150);
						PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1, 0);
						PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14, 0);
						DRV_TMR1_Start();
						DRV_OC1_Start();
						DRV_OC0_Start();
						motorData.duration = motorData.rxMessage.duration;
						break;
					}

					
					/* The default state should never be executed. */
					default:
					{
						/* TODO: Handle error in application's state machine. */
						break;
					}
				}	//end switch
			}
		}
		else
		{
			crash("No Motor MsgQ");
		}
	}
}
 

/*******************************************************************************
 End of File
 */
