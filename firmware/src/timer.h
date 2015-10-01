/* 
 * File:   app1_public.h
 * Author: lucun_000
 *
 * Created on September 17, 2015, 8:26 AM
 */

#ifndef APP1_PUBLIC_H
#define	APP1_PUBLIC_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "motor_public.h"


#ifdef	__cplusplus
extern "C" {
#endif


void theTimerInit(int msCount);
void theTimerCallback(TimerHandle_t pxTimer);


#ifdef	__cplusplus
}
#endif

#endif	/* APP1_PUBLIC_H */

