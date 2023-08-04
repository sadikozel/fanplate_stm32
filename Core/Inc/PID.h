/*
 * PID.h
 *
 *  Created on: Jun 1, 2022
 *      Author: SOYLU
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"


typedef struct{
	float Kp;
	float Ki;
	float Kd;

	float pTerm;
	float iTerm;
	float dTerm;

	float iSum;
	float iMax;

	float error;
	float lastError;
	float lastPosition;
}PID_t;

void PID_init(PID_t* pid, float Kp, float Ki, float Kd, uint32_t iMax);
float PID_iterate(PID_t* pid, float target, float current, float dt);

void PID_set_Kp(PID_t* pid, float Kp);
void PID_set_Ki(PID_t* pid, float Ki);
void PID_set_Kd(PID_t* pid, float Kd);

#endif /* INC_PID_H_ */
