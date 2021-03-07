/*
 * PID.cpp
 *
 *  Created on: 2019. 5. 23.
 *      Author: user
 */

#include "PID.h"


float update(PID *target,float current_Val) {
	target->error = target->set_point - current_Val;

	float P_Val = target->Kp * target->error;
	float D_Val = target->Kd * (target->error - target->Derivator);
	float Derivator = target->error;

	if (target->Integrator > target->Integrator_max)
		target->Integrator = target->Integrator_max;
	else if (target->Integrator < target->Integrator_min)
		target->Integrator = target->Integrator_min;

	float I_Val = target->Integrator * target->Ki;

	float PID_Val = P_Val + I_Val + D_Val;

	target->Integrator = target->Integrator + target->error;

	return PID_Val;
}

void initPID(PID *target,float P, float I, float D) {
	// TODO Auto-generated constructor stub
	target->Kp = P;
	target->Ki = I;
	target->Kd = D;

	target->Integrator = 0;
	target->Derivator = 0;
	target->Integrator_max = 1000;
	target->Integrator_min = -1000;

	target->set_point = 0;
	target->error = 0;
}
