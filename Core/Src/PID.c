/*
 * PID.c
 *
 *  Created on: Jun 1, 2022
 *      Author: SOYLU
 */

#include "PID.h"

void PID_init(PID_t* pid, float Kp, float Ki, float Kd, uint32_t iMax){
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;

	pid->pTerm = 0;
	pid->iTerm = 0;
	pid->dTerm = 0;

	pid->iSum = 0;
	pid->iMax = iMax;

	pid->error = 0;
	pid->lastError = 0;
	pid->lastPosition = 0;
}

float PID_iterate(PID_t* pid, float target, float current, float dt){
	pid->error = target - current;

	pid->pTerm = pid->Kp * pid->error;

	pid->iSum = pid->iSum + (((pid->error + pid->lastError) / 2) * dt);

	if(pid->iSum > pid->iMax){
		pid->iSum = pid->iMax;
	}
	else if(pid->iSum < -pid->iMax){
		pid->iSum = -pid->iMax;
	}

	pid->iTerm = pid->Ki * pid->iSum;

	pid->dTerm = pid->Kd * ((current - pid->lastPosition) / dt);

	pid->lastError = pid->error;
	pid->lastPosition = current;

	return pid->pTerm + pid->iTerm - pid->dTerm;

}

void PID_set_Kp(PID_t* pid, float Kp){
	pid->Kp = Kp;
}

void PID_set_Ki(PID_t* pid, float Ki){
	pid->Ki = Ki;
}

void PID_set_Kd(PID_t* pid, float Kd){
	pid->Kd = Kd;
}


