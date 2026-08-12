/*
 * motor.h
 *
 *  Created on: Jul 13, 2026
 *      Author: gurus
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <math.h>
#include <stdio.h>
#include <stdint.h>

typedef struct
{
	uint16_t pwm;
}Motor_t;

void Motor_Write(Motor_t motors[4]);

#endif
