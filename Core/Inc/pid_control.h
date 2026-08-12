/*
 * pid_control.h
 *
 *  Created on: Jun 21, 2026
 *      Author: gurus
 */

#ifndef INC_CONTROL_SYSTEM_H_
#define INC_CONTROL_SYSTEM_H_
#define MAX_CONTROL_SIGNAL 400
#define MIN_CONTROL_SIGNAL -400

#include <stdint.h>
#include <stdio.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float dt;

    float integral;
    float previousError;
    float out_min, out_max;
} PID_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max);
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt);


#endif /* INC_CONTROL_SYSTEM_H_ */
