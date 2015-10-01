#include "timer.h"

void theTimerInit(int msCount)
{
	TickType_t timertime = (msCount/portTICK_PERIOD_MS);
	TimerHandle_t theTimer = xTimerCreate("theTimer", timertime , pdTRUE, 0, theTimerCallback );
	if( xTimerStart(theTimer, 0) != pdPASS )
	{
		debugU("Timer failed to start");
	}

}

void theTimerCallback(TimerHandle_t pxTimer)
{
	motor_durationTick();
}
