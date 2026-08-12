/*
 * altitude.c
 *
 *  Created on: Jul 12, 2026
 *      Author: gurus
 */

#include <stdio.h>
#include <stdint.h>
#include <altitude.h>

float dt=0.004;
static float thetaHat_deg = 0.0f;
static float phiHat_deg = 0.0f;
static float phiUncertainty   = 4.0f;
static float thetaUncertainty = 4.0f;


void kalmanfilter(float *State, float *Uncertainty, float Input, float Measurement, float dt)
{
    *State = *State + dt * Input;
    *Uncertainty = *Uncertainty + dt * dt * 4.0f * 4.0f;

    float Kalman_Gain = (*Uncertainty) / ((*Uncertainty) + 3.0f * 3.0f);

    *State = *State + Kalman_Gain * (Measurement - *State);
    *Uncertainty = (1.0f - Kalman_Gain) * (*Uncertainty);
}


void Altitude_Update(IMU_Data_t *imu, Altitude_t *altitude, float dt)
{

    //printf("Acc_x:%.2f,Acc_y:%.2f,Acc_z:%.2f  Gyro_x:%.2f,Gyro_y:%.2f,Gyro_z:%.2f\r\n",MPU6050.acc_x, MPU6050.acc_y, MPU6050.acc_z,
	//			  MPU6050.gyro_x, MPU6050.gyro_y, MPU6050.gyro_z);
	//get filtered acc data from mpu
	float ax_msp2 = imu->acc_x;
	float ay_msp2 = imu->acc_y;
	float az_msp2 = imu->acc_z;

	//I.) angle estimation using ACCELEROMTER data in RAD/S
	float phiHat_acc_deg  = atanf(ay_msp2 / az_msp2) * RAD_TO_DEG;
	float thetaHat_acc_deg = atan2f(-ax_msp2, sqrtf(ay_msp2*ay_msp2 + az_msp2*az_msp2)) * RAD_TO_DEG;

	//get filtered gyro data from mpu
	float p_rps = imu->gyro_x;
	float q_rps = imu->gyro_y;
	float r_rps = imu->gyro_z;

	float theta_rad = thetaHat_deg * DEG_TO_RAD;
	float phi_rad   = phiHat_deg   * DEG_TO_RAD;

	//calculate euler rates
	//II.) angle estimation from GYROSCOPE data in DEG/S
	float phidot   = p_rps + (tanf(theta_rad) * sinf(phi_rad) * q_rps) +
							 (tanf(theta_rad) * cosf(phi_rad) * r_rps);
	float thetadot =         cosf(phi_rad) * q_rps - sinf(phi_rad) * r_rps;

	kalmanfilter(&phiHat_deg, &phiUncertainty, phidot, phiHat_acc_deg, dt);
	kalmanfilter(&thetaHat_deg, &thetaUncertainty, thetadot, thetaHat_acc_deg, dt);

	altitude->angle_roll = phidot;
	altitude->angle_pitch= thetadot;
	altitude->angle_yaw += imu->gyro_z * dt;

}



