/*
 * pid.c
 *
 *  Created on: Apr 8, 2022
 *      Author: lukas
 */

#include "pid.h"

void PIDController_Init(PIDController *pid, const float kP, const float kI, const float kD, const float sampleTime, const float tau,
						const float limMin, const float limMax, const float limMinInt, const float limMaxInt) {

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;

	pid->Kp = kP;
	pid->Ki = kI;
	pid->Kd = kD;

	pid->T = sampleTime;

	pid->tau = tau;

	pid->limMin = limMin;
	pid->limMax = limMax;

	pid->limMinInt = limMinInt;
	pid->limMaxInt = limMaxInt;

}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

	/*
	* Error signal
	*/
    float error = setpoint - measurement;


	/*
	* Proportional
	*/
    float proportional = pid->Kp * error;


	/*
	* Integral
	*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }


	/*
	* Derivative (band-limited differentiator)
	*/

    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) + (2.0f * pid->tau - pid->T) * pid->differentiator) / (2.0f * pid->tau + pid->T);


	/*
	* Compute output and apply limits
	*/
    //pid->differentiator = pid->Kd*(measurement - pid->prevMeasurement);

    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

	/* Return controller output */
    return pid->out;

}
