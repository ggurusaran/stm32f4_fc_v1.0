/*
 * rc_receiver.h
 *
 *  Created on: Jul 12, 2026
 *      Author: gurus
 */

#ifndef INC_RC_RECEIVER_H_
#define INC_RC_RECEIVER_H_

#include <stdint.h>
#include <stdio.h>

/*Raw input from RC Transmitter from (1000-2000us)*/
typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float throttle;
} RC_Input_t;


/*Desired values of roll, pitch and yaw after calculation in terms of angle*/
typedef struct
{
    float throttle;    // 0.0 to 1.0
    float roll;        // degrees or deg/s
    float pitch;
    float yaw;
} RC_Command_t;


void RC_Process(const RC_Input_t *raw, RC_Command_t *cmd);


#endif /* INC_RC_RECEIVER_H_ */
