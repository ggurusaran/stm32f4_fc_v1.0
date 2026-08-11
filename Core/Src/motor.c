/*
 * motor.c
 *
 *  Created on: Jul 13, 2026
 *      Author: gurus
 */


#include "main.h"
#include "motor.h"

extern TIM_HandleTypeDef htim1;

void Motor_Write(Motor_t motors[4])
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motors[0].pwm);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motors[1].pwm);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, motors[2].pwm);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, motors[3].pwm);
}
