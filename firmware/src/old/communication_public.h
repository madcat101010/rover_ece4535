/* 
 * File:   communication_public.h
 * Author: lucun_000
 *
 * Created on September 27, 2015, 7:49 PM
 */

#ifndef COMMUNICATION_PUBLIC_H
#define	COMMUNICATION_PUBLIC_H	

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "FreeRTOS.h"


#ifdef	__cplusplus
extern "C" {
#endif

//void communication_sendmsg(int command, int duration);
void communication_sendmsgISR(char rxByte);

//void communication_sendmsg(int command, int duration);
//void communication_sendmsgISR(int command, int duration);
void communication_sendIntMsg(int left, int right);
void communication_getByteISR();
bool communication_IntQueueEmptyISR();

#ifdef	__cplusplus
}
#endif

#endif	/* COMMUNICATION_PUBLIC_H */

