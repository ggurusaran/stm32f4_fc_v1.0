/*
 * flight_controller.h
 *
 *  Created on: Jul 11, 2026
 *      Author: gurus
 */

#ifndef INC_FLIGHT_CONTROLLER_H_
#define INC_FLIGHT_CONTROLLER_H_

#include <attitude.h>
#include <math.h>
#include <motor.h>
#include <stdio.h>
#include <stdint.h>
#include <mpu6050.h>
#include <pid_control.h>
#include <bmp280.h>
#include <mpu6050.h>
#include <rc_receiver.h>
#include <pid_control.h>

/* Angle PID */
#define ROLL_ANGLE_KP    4.0f
#define ROLL_ANGLE_KI    0.0f
#define ROLL_ANGLE_KD    0.10f

#define PITCH_ANGLE_KP   4.0f
#define PITCH_ANGLE_KI   0.0f
#define PITCH_ANGLE_KD   0.10f

/* Rate PID */
#define ROLL_RATE_KP     0.15f
#define ROLL_RATE_KI     0.02f
#define ROLL_RATE_KD     0.003f

#define PITCH_RATE_KP    0.15f
#define PITCH_RATE_KI    0.02f
#define PITCH_RATE_KD    0.003f

#define YAW_RATE_KP      0.20f
#define YAW_RATE_KI      0.00f
#define YAW_RATE_KD      0.000f

#define MOTOR_MAX_PWM    2000.0f
#define MOTOR_MIN_PWM    1150.0f



typedef struct
{
    IMU_Data_t imu;
    Attitude_t attitude;
    RC_Input_t rawPWM;
    RC_Command_t rc_cmd;

    PID_t rollAnglePID;
    PID_t pitchAnglePID;

    PID_t rollRatePID;
    PID_t pitchRatePID;
    PID_t yawRatePID;

    float rollOutput;
    float pitchOutput;
    float yawOutput;

    float desiredRollRate;
    float desiredPitchRate;

    Motor_t motors[4];

} FlightController_t;

void Controller_Init(FlightController_t *fc);
void pid_outer_loop(FlightController_t *fc);
void pid_inner_loop(FlightController_t *fc);
void Motor_mix(FlightController_t *fc);


#endif /* INC_FLIGHT_CONTROLLER_H_ */
