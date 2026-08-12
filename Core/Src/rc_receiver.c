/*
 * rc_receiver.c
 *
 *  Created on: Jul 12, 2026
 *      Author: gurus
 */


#include <rc_receiver.h>

#define RC_MIN      	1000.0f
#define RC_CENTER       1500.0f
#define RC_MAX       	2000.0f

#define MAX_ROLL_ANGLE   30.0f    // degrees
#define MAX_PITCH_ANGLE  30.0f    // degrees
#define MAX_YAW_RATE     180.0f   // deg/s
#define RC_DEADBAND      20.0f    // us

void RC_Process(const RC_Input_t *raw, RC_Command_t *cmd)
{
    /* Throttle: 1000-2000 us -> 0.0-1.0 */
    cmd->throttle = (raw->throttle - RC_MIN) / (RC_MAX - RC_MIN);

    /* Roll/Pitch: 1000-2000 us -> -30° to +30° */
    cmd->roll  = ((raw->roll - RC_CENTER) / 500.0f) * MAX_ROLL_ANGLE;

    cmd->pitch = ((raw->pitch - RC_CENTER) / 500.0f) * MAX_PITCH_ANGLE;

    /* Yaw: 1000-2000 us -> -180 to +180 deg/s */
    cmd->yaw   = ((raw->yaw - RC_CENTER) / 500.0f) * MAX_YAW_RATE;

}

