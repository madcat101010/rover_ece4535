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
	void motor_LEncode();
	void motor_REncode();
	int motor_PIDLEncode();
	int motor_PIDREncode();
	int motor_ResetLEncodeTotal();
	int motor_ResetREncodeTotal();
	void motor_clearLREncode();

	
#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_PUBLIC_H */

