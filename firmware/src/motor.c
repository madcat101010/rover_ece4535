
#include "motor.h"
#include "peripheral/oc/plib_oc.h"

#define MOTORQUEUELENGTH 10
#define MAXSPEED 32.50			//ABS MAX IS 55
#define MAXSPEEDOCSET 180
#define PFACTOR 2.0
#define IFACTOR 1.0
#define PFACTORTURN 5.0
#define IFACTORTURN 5.0
//#define MMPERTICK 0.1350	//0.07476 for both edge, 0.1495 for single edge
#define MMPERTICK 0.0655	//0.0665
#define WHEELRADIUS 5

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
void motor_sendmsg(int command, int int2)
{
	MOTOR_MESSAGE theMessage;
	theMessage.SeqNum = 0;//motorData.TxSeqNum;
	theMessage.command = command;
	theMessage.int2 = int2;
	if(motorData.theQueue != 0)
	{
		if( xQueueSend(motorData.theQueue, (void*)&(theMessage), portMAX_DELAY))
		{
			motorData.TxSeqNum++;
		}
	}
}

void motor_sendmsgISR(int command, int int2)
{
	MOTOR_MESSAGE theMessage;
	theMessage.SeqNum = motorData.TxSeqNum;
	theMessage.command = command;
	theMessage.int2 = int2;
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
	motorData.n++;
/*	if( motorData.state != 0)
	{
		if( motorData.duration == 0)
		{
			motor_sendmsgISR(0, 0);
//			motor_sendmsgISR(1, 1000);
//			motor_sendmsgISR(3,1000);
		}
		else
			motorData.duration--;
	}
//*/
}

void motor_LEncode()
{
	motorData.LEncode++;
	motorData.LEncodeTotal++;
	motorData.LTestTick++;
}

void motor_REncode()
{
	motorData.REncode++;
	motorData.REncodeTotal++;
	motorData.RTestTick++;
}

void motor_clearLREncode()
{
	motorData.REncode = 0;
	motorData.LEncode = 0;
}

int motor_PIDLEncode()
{
	float PErrorFix;
	float IErrorFix;
	int test;
	motorData.LDist = (float)motorData.LEncode * MMPERTICK;	//mm traveled
	motorData.LMes = motorData.LDist * 20.0;				//mm/sec
	debugU("\nLeft:");
	debugUFloat(motorData.LMes);
	if(motorData.state != 3)
	{
		float error = motorData.LSet - motorData.LMes;
		motorData.LAvg = ( (error) + (motorData.LAvg * (motorData.n - 1)) ) / motorData.n;	//error in mm/sec average
		PErrorFix = PFACTOR * error;
		IErrorFix = IFACTOR * (motorData.LAvg);
		float total = PErrorFix + IErrorFix;
		if(total < 0.0)
			test = (int)(total -0.5);
		else
			test = (int)(total +0.5);
		motorData.LOCSet = motorData.LOCSet + test;
	}
	else
	{
		if(motorData.RMes == 0)	//update RMes if it DNE yet.
		{
			motorData.RMes = (float)motorData.REncode * MMPERTICK * 20.0;	
		}
		float currRatio = (float)motorData.LMes / (float)motorData.RMes;
		float error = (motorData.ratio - currRatio); //*4.0 here orig...
		motorData.LAvg = ( (error) + (motorData.LAvg * (motorData.n - 1)) ) / motorData.n;	//error in mm/sec average
		PErrorFix = PFACTORTURN * error;
		IErrorFix = IFACTORTURN * (motorData.LAvg);
		float total = PErrorFix + IErrorFix;
		if(total < 0.0)
			test = (int)(total -0.5);
		else
			test = (int)(total +0.5);
		motorData.LOCSet = motorData.LOCSet + test;
	}
	if(motorData.LOCSet < 0)
		motorData.LOCSet = 0;
//	if(motorData.LOCSet > 600 && motorData.state == 2 )
//		motorData.LOCSet = 600;
	PLIB_OC_PulseWidth16BitSet(OC_ID_2, motorData.LOCSet);
///*
	debugUFloat(motorData.LAvg);
	debugUFloat(PErrorFix);
	debugUFloat(IErrorFix);
	debugUInt(motorData.LOCSet);
	debugUInt(test);//*/
	return motorData.LEncode;
}

int motor_PIDREncode()
{
	float PErrorFix;
	float IErrorFix;
	int test;
	motorData.RDist = (float)motorData.REncode * MMPERTICK;
	motorData.RMes = motorData.RDist * 20.0;
	debugU("\nRight:");
	debugUFloat(motorData.RMes);
	if(motorData.state != 2)
	{
		float error = motorData.RSet - motorData.RMes;
		motorData.RAvg = ( (error) + (motorData.RAvg * (motorData.n - 1)) ) / motorData.n;	//error in mm/sec average
		PErrorFix = PFACTOR * error;
		IErrorFix = IFACTOR * (motorData.RAvg);
		float total = PErrorFix + IErrorFix;
		if(total < 0.0)
			test = (int)(total -0.5);
		else
			test = (int)(total +0.5);
		motorData.ROCSet = motorData.ROCSet + test;
	}
	else
	{
		float currRatio = (float)motorData.RMes / (float)motorData.LMes;
		float error = 4.0*(motorData.ratio - currRatio);
		motorData.RAvg = ( (error) + (motorData.RAvg * (motorData.n - 1)) ) / motorData.n;	//error in mm/sec average
		PErrorFix = PFACTORTURN * error;
		IErrorFix = IFACTORTURN * (motorData.RAvg);
		float total = PErrorFix + IErrorFix;
		if(total < 0.0)
			test = (int)(total -0.5);
		else
			test = (int)(total +0.5);
		motorData.ROCSet = motorData.ROCSet + test;

	}
	if(motorData.ROCSet < 0)
		motorData.ROCSet = 0;
//	if(motorData.ROCSet > 600 && motorData.state == 3 )
//		motorData.ROCSet = 600;
	PLIB_OC_PulseWidth16BitSet(OC_ID_1, motorData.ROCSet);
///*
	debugUFloat(motorData.RAvg);
	debugUFloat(PErrorFix);
	debugUFloat(IErrorFix);
	debugUInt(motorData.ROCSet);
	debugUInt(test);//*/
	return motorData.REncode;
}

int motor_ResetLEncodeTotal()
{
	int ret = motorData.LEncodeTotal;
	motorData.LEncodeTotal = 0;
	return ret;
}

int motor_ResetREncodeTotal()
{
	int ret = motorData.REncodeTotal;
	motorData.REncodeTotal = 0;
	return ret;
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
	motorData.LEncode = 0;
	motorData.REncode = 0;
	motorData.REncodeTotal = 0;
	motorData.LEncodeTotal = 0;
	motorData.LDist = 0;
	motorData.RDist = 0;
	motorData.LSet = 0;
	motorData.RSet = 0;
	motorData.LMes = 0;
	motorData.RMes = 0;
	motorData.LAvg = 0.0;
	motorData.RAvg = 0;
	motorData.LOCSet = 0;
	motorData.ROCSet = 0;
	motorData.ratio = 0;
	motorData.RTestTick = 0;
	motorData.LTestTick = 0;
	motorData.n = 0;
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
	DRV_TMR0_Stop();	//pwm timer
	DRV_TMR1_Stop();	//encoder timer 5ms
	DRV_TMR2_Stop();	//duration timer 1ms
	
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
	//debugU("RUNNING");
//		motor_sendmsg(1, 1000);
//	motor_sendmsg(1, 1615);		
//	motor_sendmsg(3, 1350);
	
	while(1)
	{
		if(motorData.theQueue != 0)
		{
			if(xQueuePeek(motorData.theQueue, (void*)&(motorData.rxMessage), portMAX_DELAY ))
			{
				xQueueReceive(motorData.theQueue, (void*)&(motorData.rxMessage), portMAX_DELAY );
				motorData.state = motorData.rxMessage.command;
//				motorData.state = 1;
//				debugUInt(motorData.state);
//				debugUInt(motorData.duration);
//				communication_sendIntMsg(motorData.rxMessage.command, motorData.rxMessage.duration);
				/* Check the application's current state. */
				switch ( motorData.state )
				{
					/* Application's initial state. */
					case MOTOR_STATE_INIT:	//state 0, do nothing
					{
						DRV_TMR0_Stop();
						DRV_TMR1_Stop();
						DRV_TMR2_Stop();
						motorData.LAvg = 0.0;
						motorData.RAvg = 0.0;
						motorData.LEncode = 0;
						motorData.REncode = 0;
						motorData.LEncodeTotal = 0;
						motorData.REncodeTotal = 0;
						motorData.n = 0;
						DRV_TMR0_CounterClear();
						DRV_TMR1_CounterClear();
						DRV_TMR2_CounterClear();
						DRV_OC1_Stop();
						DRV_OC0_Stop();
						break;
					}
					case MOTOR_STATE_STRAIGHT: //state 1, go straight.
					{
						debugU("Straight");
						//stop command
						if(motorData.rxMessage.int2 == 0)
						{
							DRV_TMR0_Stop();
							DRV_TMR1_Stop();
							DRV_TMR2_Stop();
							DRV_TMR0_Stop();
							DRV_TMR1_Stop();
							DRV_TMR2_Stop();
							DRV_OC1_Stop();
							DRV_OC0_Stop();
							float time = (float)motorData.n * 0.05;
							float Ldistance = MMPERTICK * motorData.LTestTick;
							float Rdistance = MMPERTICK * motorData.RTestTick;
							motorData.LTestTick = 0;
							motorData.RTestTick = 0;
							debugUFloat(time);
							debugU("Right:\n");
							debugUFloat(motorData.RSet);
							debugUFloat(Rdistance);
							debugU("Left:\n");
							debugUFloat(motorData.LSet);
							debugUFloat(Ldistance);
							debugUFloat(motorData.ratio);
							motorData.LSet = 0.0;
							motorData.RSet = 0.0;
							motorData.LAvg = 0.0;
							motorData.RAvg = 0.0;
							motorData.LEncode = 0;
							motorData.REncode = 0;
							motorData.LEncodeTotal = 0;
							motorData.REncodeTotal = 0;
							motorData.n = 0;
							DRV_TMR0_CounterClear();
							DRV_TMR1_CounterClear();
							DRV_TMR2_CounterClear();
							DRV_TMR0_CounterClear();
							DRV_TMR1_CounterClear();
							DRV_TMR2_CounterClear();
						}
						else	//just go straight
						{
							motorData.LSet = MAXSPEED;
							motorData.RSet = MAXSPEED;
							motorData.LOCSet = MAXSPEEDOCSET;
							motorData.ROCSet = MAXSPEEDOCSET;
							PLIB_OC_PulseWidth16BitSet(OC_ID_1, motorData.ROCSet);
							PLIB_OC_PulseWidth16BitSet(OC_ID_2, motorData.LOCSet);
							PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1, 0);
							PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14, 0);
							DRV_TMR0_Stop();
							DRV_TMR1_Stop();
							DRV_TMR2_Stop();
							motorData.LAvg = 0.0;
							motorData.RAvg = 0.0;
							motorData.LEncode = 0;
							motorData.REncode = 0;
							motorData.LEncodeTotal = 0;
							motorData.REncodeTotal = 0;
							motorData.n = 0;
							DRV_TMR0_CounterClear();
							DRV_TMR1_CounterClear();
							DRV_TMR2_CounterClear();
							DRV_OC1_Start();
							DRV_OC0_Start();
							DRV_TMR0_Start();
							DRV_TMR1_Start();
							DRV_TMR2_Start();
						}
						break;
					}

					case MOTOR_STATE_TURNRIGHT: //state 2, turn right
					{
						debugU("Right");
						DRV_TMR0_Stop();
						DRV_TMR1_Stop();
						DRV_TMR2_Stop();
						motorData.LAvg = 0.0;
						motorData.RAvg = 0.0;
						motorData.LEncode = 0;
						motorData.REncode = 0;
						motorData.LEncodeTotal = 0;
						motorData.REncodeTotal = 0;
						motorData.n = 0;
						DRV_TMR0_CounterClear();
						DRV_TMR1_CounterClear();
						DRV_TMR2_CounterClear();
						PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1, 0);
						PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14, 0);
						float angular = MAXSPEED / (float)motorData.rxMessage.int2;
						int outRad = motorData.rxMessage.int2 + 5;
						motorData.LSet = angular * (float)outRad;
						//motorData.LSet = MAXSPEED;
						motorData.LOCSet = (int)(motorData.LSet * 6);
						PLIB_OC_PulseWidth16BitSet(OC_ID_2, motorData.LOCSet);// - 100);
						if(outRad < 11)
						{
							motorData.RSet = 0.0;
							motorData.ROCSet = 0;
//							PLIB_OC_PulseWidth16BitSet(OC_ID_1, motorData.ROCSet);
//							DRV_OC0_Start();
							DRV_OC0_Stop();
						}
						else
						{
							float temp = (float)outRad;
							motorData.ratio = ((temp - 10.0) / temp);
							motorData.RSet = (temp - 10.0) * angular;
//							motorData.RSet = ( motorData.ratio * (MAXSPEED)); 
							motorData.ROCSet = (int)(motorData.RSet * 6); //motorData.RSet * 5.5;
							PLIB_OC_PulseWidth16BitSet(OC_ID_1, motorData.ROCSet);
							DRV_OC0_Start();
						}
						DRV_OC1_Start();
						DRV_TMR0_Start();
						DRV_TMR1_Start();
						DRV_TMR2_Start();
						break;
					}
					
					case MOTOR_STATE_TURNLEFT: //state 3 turn left.
					{
						debugU("Left");
						DRV_TMR0_Stop();
						DRV_TMR1_Stop();
						DRV_TMR2_Stop();
						motorData.LAvg = 0.0;
						motorData.RAvg = 0.0;
						motorData.LEncode = 0;
						motorData.REncode = 0;
						motorData.LEncodeTotal = 0;
						motorData.REncodeTotal = 0;
						motorData.n = 0;
						DRV_TMR0_CounterClear();
						DRV_TMR1_CounterClear();
						DRV_TMR2_CounterClear();
						PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1, 0);
						PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14, 0);

						float angular = MAXSPEED / (float)motorData.rxMessage.int2;
						int outRad = motorData.rxMessage.int2 + 5;
						motorData.RSet = angular * (float)outRad;
						motorData.ROCSet = (int)(motorData.RSet * 6);

//						motorData.RSet = MAXSPEED;
//						motorData.ROCSet = MAXSPEEDOCSET;
						PLIB_OC_PulseWidth16BitSet(OC_ID_1, motorData.ROCSet);
						if(outRad < 11)
						{
							motorData.LSet = 0.0;
							motorData.LOCSet = 0;
//							PLIB_OC_PulseWidth16BitSet(OC_ID_2, motorData.LOCSet);
//							DRV_OC1_Start();
							DRV_OC1_Stop();
						}
						else
						{
							float temp = (float)outRad;
							motorData.ratio = ((temp - 10.0) / temp);
							motorData.LSet = (temp - 10.0) * angular;
							motorData.LOCSet = (int)(motorData.LSet * 6); //motorData.RSet * 5.5;
							PLIB_OC_PulseWidth16BitSet(OC_ID_2, motorData.LOCSet);
							DRV_OC1_Start();
						}
						DRV_OC0_Start();
						DRV_TMR0_Start();
						DRV_TMR1_Start();
						DRV_TMR2_Start();
						break;
					}


					/* The default state should never be executed. */
					default:
					{
						DRV_TMR0_Stop();
						DRV_TMR2_Stop();
						DRV_TMR0_CounterClear();
						DRV_TMR2_CounterClear();
						DRV_OC1_Stop();
						DRV_OC0_Stop();
						motorData.duration = 0;
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
/*
 
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
 
 
// */

/*
 						motorData.LSet = MAXSPEED;
						motorData.LOCSet = MAXSPEEDOCSET;
						PLIB_OC_PulseWidth16BitSet(OC_ID_2, motorData.LOCSet - 100);
						int outRad = motorData.rxMessage.int2 + 5;
						if(outRad < 11)
						{
							motorData.RSet = 0.0;
							motorData.ROCSet = 0;
//							PLIB_OC_PulseWidth16BitSet(OC_ID_1, motorData.ROCSet);
//							DRV_OC0_Start();
							DRV_OC0_Stop();
						}
						else
						{
							float temp = (float)outRad;
							motorData.ratio = ((temp - 10.0) / temp);
							motorData.RSet = ( motorData.ratio * (MAXSPEED)); 
							motorData.ROCSet = (int)(motorData.ratio * (float)MAXSPEEDOCSET); //motorData.RSet * 5.5;
							PLIB_OC_PulseWidth16BitSet(OC_ID_1, motorData.ROCSet);
							DRV_OC0_Start();
						}
						DRV_OC1_Start();
						DRV_TMR0_Start();
						DRV_TMR1_Start();
						DRV_TMR2_Start();
						break;

 
 
 
 
 
 */