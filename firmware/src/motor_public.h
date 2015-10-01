/* 
 * File:   motor_public.h
 * Author: lucun_000
 *
 * Created on September 27, 2015, 7:49 PM
 */

#ifndef MOTOR_PUBLIC_H
#define	MOTOR_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif

	void motor_sendmsg(int command, int duration);
	void motor_sendmsgISR(int command, int duration);
	void motor_durationTick();
	
#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_PUBLIC_H */

