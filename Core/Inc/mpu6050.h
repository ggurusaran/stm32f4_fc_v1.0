/*
 * mpu6050.h
 *
 *  Created on: May 28, 2026
 *      Author: gurus
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"
#include <stdbool.h>

typedef struct _MPU6050
{
	short temperature_raw;
	short acc_x_raw;
	short acc_y_raw;
	short acc_z_raw;
	short gyro_x_raw;
	short gyro_y_raw;
	short gyro_z_raw;

 	float temperature;

	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;

	float acc_x_bias;
	float acc_z_bias;
	float acc_y_bias;

 	double Accel_Z_Calib;
	double roll_calibration_value;
	double pitch_calibration_value;
	double yaw_calibration_value;
	float temp_calibration_value;

} IMU_Data_t;

extern IMU_Data_t imu;

#define COMP_FILT_ALPHA    0.05000000f

#define MPU6050_ADDR       0x68
#define GYRO_RATE 		   65.5f
#define ACCELERATION_RATE  8192.0f

#define FS_GYRO_250 	   0x00
#define FS_GYRO_500 	   0x08
#define FS_GYRO_1000 	   0x09
#define FS_GYRO_2000 	   0x0A

#define FS_ACC_2G 	   	   0x00
#define FS_ACC_4G	   	   0x08
#define FS_ACC_8G 	 	   0x09
#define FS_ACC_16G 	  	   0x0A


#define REG_CONFIG		   0X1A
#define REG_CONFIG_GYRO	   0X1B
#define REG_CONFIG_ACC     0X1C
#define REG_DATA_ACC	   0X3B
#define REG_DATA_GYRO	   0X43
#define REG_PWR_MGMT_1	   0X6B



HAL_StatusTypeDef  MPU6050_Writebyte(uint8_t reg_addr, uint8_t val);
HAL_StatusTypeDef  MPU6050_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t* data);
HAL_StatusTypeDef  MPU6050_Readbyte(uint8_t reg_addr, uint8_t* data);
HAL_StatusTypeDef  MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t* data);

void mpu6050_init();
bool calibrate_gyro(IMU_Data_t *imu);
bool calibrate_ACC(IMU_Data_t *imu);
void MPU6050_Read_All(IMU_Data_t *imu);
void kalmanfilter(float *State, float *Uncertainty, float Input, float Measurement, float dt);
#endif /* INC_MPU6050_H_ */
