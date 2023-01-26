/*
 * pid.h
 *
 *  Created on: Apr 8, 2022
 *      Author: lukas
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;

	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

} PIDController;

void PIDController_Init(PIDController *pid, const float kP, const float kI, const float kD, const float sampleTime, const float tau,
						const float limMin, const float limMax, const float limMinInt, const float limMaxInt);

float PIDController_Update(PIDController *pid, float setpoint, float measurement);


#endif /* INC_PID_H_ */
