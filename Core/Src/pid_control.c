/*
 * pid_control.c
 *
 *  Created on: Jun 21, 2026
 *      Author: gurus
 */

#include <pid_control.h>
#include <mpu6050.h>
#include <stdint.h>
#include <stdio.h>
#include <main.h>


void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max)
{
	pid->kp = Kp;
	pid->ki = Ki;
	pid->kd = Kd;

	pid->integral = 0.0f;
	pid->previousError = 0.0f;
	pid->out_min = out_min; pid->out_max = out_max;
}

float PID_Update(PID_t *pid,
                 float setpoint,
                 float measurement,
                 float dt)
{
    float error = setpoint - measurement;

    float PTerm = pid->kp * error;

    /*
     * Calculate derivative
     */
    float derivative = (error - pid->previousError) / dt;

    float DTerm = -pid->kd * derivative;

    /*
     * Candidate integral
     */
    float newIntegral =
        pid->integral + error * dt;

    float ITerm = pid->ki * newIntegral;


    if (ITerm > 400.0f)
        ITerm = 400.0f;
    else if (ITerm < -400.0f)
        ITerm = -400.0f;

    float output = PTerm + ITerm + DTerm;

    if (output > pid->out_max)
        output = pid->out_max;
    else if (output < pid->out_min)
        output = pid->out_min;
    else
        pid->integral = newIntegral;

    pid->previousError = error;

    return output;
}

void PID_Reset(PID_t *pid)
{
	pid->kp=0;
	pid->ki=0;
	pid->kd=0;
}




