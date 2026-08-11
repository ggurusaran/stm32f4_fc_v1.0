/*
 * altitude.h
 *
 *  Created on: Jul 12, 2026
 *      Author: gurus
 */

#ifndef INC_ALTITUDE_H_
#define INC_ALTITUDE_H_

#define DEG_TO_RAD  (3.14159265f / 180.0f)
#define RAD_TO_DEG  (180.0f / 3.14159265f)
#define G_MSP2	    (9.8)

#include <mpu6050.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

typedef struct
{
	float angle_roll;
	float angle_pitch;
	float angle_yaw;
}Altitude_t;

void kalmanfilter(float *State, float *Uncertainty, float Input, float Measurement, float dt);
void Altitude_Update(IMU_Data_t *imu, Altitude_t *attitude, float dt);



#endif /* INC_ALTITUDE_H_ */
