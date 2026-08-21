	/*
 * flight_controller.c
 *
 *  Created on: Jul 11, 2026
 *      Author: gurus
 */


#include <flight_controller.h>


void Controller_Init(FlightController_t *fc)
{
	PID_Init(&fc->rollAnglePID,ROLL_ANGLE_KP,ROLL_ANGLE_KI,ROLL_ANGLE_KD,-200.0f, 200.0f);
	PID_Init(&fc->pitchAnglePID,PITCH_ANGLE_KP,PITCH_ANGLE_KI,PITCH_ANGLE_KD,-200.0f, 200.0f);

	PID_Init(&fc->rollRatePID,ROLL_RATE_KP,ROLL_RATE_KI,ROLL_RATE_KD,-400.0f, 400.0f);
	PID_Init(&fc->pitchRatePID,PITCH_RATE_KP,PITCH_RATE_KI,PITCH_RATE_KD,-400.0f, 400.0f);
	PID_Init(&fc->yawRatePID,YAW_RATE_KP,YAW_RATE_KI,YAW_RATE_KD,-400.0f, 400.0f);
}

//Compute PID for Roll, pitch, yaw
void pid_outer_loop(FlightController_t *fc)
{
    /* Outer loop */
    fc->desiredRollRate  =
        PID_Update(&fc->rollAnglePID,
                   fc->rc_cmd.roll,
                   fc->attitude.angle_roll,
                   0.004);

    fc->desiredPitchRate =
        PID_Update(&fc->pitchAnglePID,
                   fc->rc_cmd.pitch,
                   fc->attitude.angle_pitch,
                   0.004);
}

void pid_inner_loop(FlightController_t *fc)
{
    /* Inner loop */
    fc->rollOutput =
        PID_Update(&fc->rollRatePID,
                   fc->desiredRollRate,
                   fc->imu.gyro_x,
                   0.001);

    fc->pitchOutput =
        PID_Update(&fc->pitchRatePID,
                   fc->desiredPitchRate,
                   fc->imu.gyro_y,
                   0.001);

    fc->yawOutput =
        PID_Update(&fc->yawRatePID,
                   fc->rc_cmd.yaw,
                   fc->imu.gyro_z,
                   0.001);
}


void Motor_mix(FlightController_t *fc)
{
	fc->motors[0].pwm = (1000.0f + (fc->rc_cmd.throttle*1000.0f)) - fc->rollOutput - fc->pitchOutput - fc->yawOutput;
	fc->motors[1].pwm = (1000.0f + (fc->rc_cmd.throttle*1000.0f)) + fc->rollOutput - fc->pitchOutput + fc->yawOutput;
	fc->motors[2].pwm = (1000.0f + (fc->rc_cmd.throttle*1000.0f)) + fc->rollOutput + fc->pitchOutput - fc->yawOutput;
	fc->motors[3].pwm = (1000.0f + (fc->rc_cmd.throttle*1000.0f)) - fc->rollOutput + fc->pitchOutput + fc->yawOutput;

	for(int i=0; i<4; i++)
	{
		if(fc->motors[i].pwm> MOTOR_MAX_PWM)
			fc->motors[i].pwm= MOTOR_MAX_PWM;
		if(fc->motors[i].pwm < MOTOR_MIN_PWM)
			fc->motors[i].pwm= MOTOR_MIN_PWM;
	}
}

