/*
 * PID.h
 *
 *  Created on: 2019. 5. 23.
 *      Author: user
 */
#ifndef PID_H_
#define PID_H_


typedef struct PIDstruct {
	float Kp;
	float Ki;
	float Kd;

	float Derivator;
	float Integrator;
	float Integrator_max;
	float Integrator_min;

	float set_point;
	float error;

}PID;

float update(PID *target,float current_Val);
void setPoint(PID *target);
void initPID(PID *target,float P, float I, float D);

#endif /* PID_H_ */
