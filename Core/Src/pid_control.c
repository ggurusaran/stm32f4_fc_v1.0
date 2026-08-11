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

float PID_Update(PID_t *pid, float setpoint, float measurement ,float dt)
{
	float error = setpoint - measurement;

	float PTerm = pid->kp * error;
	pid->integral = (pid->kp * error);
	float ITerm = pid->ki * pid->integral;
	if(ITerm>400)
	{
		ITerm = 400;
	}
	else if(ITerm<-400)
	{
		ITerm = -400;
	}
	float DTerm = pid->kd * (error - pid->previousError) / dt;
	pid->previousError = error;
	float output = PTerm + ITerm + DTerm;

	if (output > pid->out_max)
	{
		output = pid->out_max;
        pid->integral -= error * dt;   // undo this step's accumulation
    }
	else if (output < pid->out_min)
	{
	    output = pid->out_min;
	    pid->integral -= error * dt;
	}
	return output;
}







