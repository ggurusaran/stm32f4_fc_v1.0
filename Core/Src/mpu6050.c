/*
 * mpu6050.c
 *
 *  Created on: May 28, 2026
 *      Author: gurus
 */
#include <math.h>
#include <stdbool.h>
#include <mpu6050.h>
#include <stdint.h>
#include <stdio.h>
#include <main.h>

#define DT  0.004
float TEMP_CALIB_COEFF_GX =2;
float TEMP_CALIB_COEFF_GY =-1.2;
float gyro_filter_coeff=0.7;

float my_alpha = 0.992;
bool set_gyro_angles;


IMU_Data_t imu;
extern I2C_HandleTypeDef hi2c1;

//SENSOR SPECIFIC READ WRITE OPERATION FOR MPU6050

HAL_StatusTypeDef  MPU6050_Writebyte(uint8_t reg_addr, uint8_t val)
{
	return HAL_I2C_Mem_Write(&hi2c1, (MPU6050_ADDR<<1), reg_addr, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

HAL_StatusTypeDef  MPU6050_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	return HAL_I2C_Mem_Write(&hi2c1, (MPU6050_ADDR<<1), reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 100);

}

HAL_StatusTypeDef  MPU6050_Readbyte(uint8_t reg_addr, uint8_t* data)
{
	return HAL_I2C_Mem_Read(&hi2c1, (MPU6050_ADDR<<1), reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

HAL_StatusTypeDef  MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	return HAL_I2C_Mem_Read(&hi2c1, (MPU6050_ADDR<<1), reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

void mpu6050_init()
{
	//configuring gyro angle and DLPF
	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, (MPU6050_ADDR<<1), 1, 100);
	if (ret == HAL_OK){
		printf("device is Ready!!\r\n");
	}else{
		printf("Device not ready!! check connection!!\r\n");
	}

	//POWER MGMT Reg to control sleep mode - IMU sensor in low power sleep mode by default, reset BIT6 to exit from sleep mode
	uint8_t temp_data = 0x00;
	ret = MPU6050_Writebyte(REG_PWR_MGMT_1, temp_data);
	if (ret == HAL_OK)
	{
		printf("Exiting from SLEEP_MODE....DONE!\r\n");
	}else{
		printf("Failed to exit SLEEP_MODE\r\n");
	}

	//2. configuring DLPF(Low Pass Filter)
	temp_data = 0X05;
	ret = MPU6050_Writebyte(REG_CONFIG, temp_data);
	if (ret == HAL_OK)
	{
		printf("writing to register DLPF_CONFIG....DONE!\r\n");
	}else{
		printf("Failed enabling the DLPF\r\n");
	}

	//configuring gyro params
	temp_data = FS_GYRO_500;
	ret = MPU6050_Writebyte(REG_CONFIG_GYRO, temp_data);
	if (ret == HAL_OK)
	{
		printf("writing to register GYRO_CONFIG....DONE!\r\n");
	}else{
		printf("Failed writing to GYRO_CONFIG\r\n");
	}

	//configuration acceleration param
    temp_data = FS_ACC_4G;
	ret = MPU6050_Writebyte(REG_CONFIG_ACC, temp_data);
	if (ret == HAL_OK)
	{
		printf("writing to register ACC_CONFIG....DONE!\r\n");
	}else{
		printf("Failed writing to ACC_CONFIG\r\n");
	}
}

bool calibrate_gyro(IMU_Data_t *imu)
{
	double sumX=0;
	double sumY=0;
	double sumZ=0;
	float sumTemp=0;
	for(int i=0; i<2000; i++)
	{
		MPU6050_Read_All(imu);
		sumX += imu->gyro_x_raw;
		sumY += imu->gyro_y_raw;
		sumZ += imu->gyro_z_raw;
		sumTemp += imu->temperature;
	}
	imu->roll_calibration_value  = sumX/(double)i;
	imu->pitch_calibration_value = sumY/(double)i;
	imu->yaw_calibration_value   = sumZ/(double)i;
	imu->temp_calibration_value  = sumTemp/i;
	return 1;
}

bool calibrate_ACC(IMU_Data_t *imu)
{
	double summationRoll=0;
	double summationPitch=0;
	double summation_Z=0;
	for(int i=0; i<2000; i++)
	{
		MPU6050_Read_All(imu);
		summationRoll += imu->acc_x;
		summationPitch += imu->acc_y;
		summation_Z  += imu->acc_z;
	}
	imu->acc_x_bias = round((summationRoll/(double)i)*10) /10;
	imu->acc_y_bias = round((summationPitch/(double)i)*10) /10;
	imu->acc_z_bias = summation_Z/i;
	return 1;
}

void MPU6050_Read_All(IMU_Data_t *imu)
{
    uint8_t data[14];
    MPU6050_Readbytes(REG_DATA_ACC, 14, data);

    /* Raw Accelerometer */
    imu->acc_x_raw = (int16_t)((data[0] << 8) | data[1]);
    imu->acc_y_raw = (int16_t)((data[2] << 8) | data[3]);
    imu->acc_z_raw = (int16_t)((data[4] << 8) | data[5]);
    /* Raw Temperature */
    imu->temperature_raw = (int16_t)((data[6] << 8) | data[7]);

    /* Raw Gyroscope */
    imu->gyro_x_raw = (int16_t)((data[8] << 8) | data[9]);
    imu->gyro_y_raw = (int16_t)((data[10] << 8) | data[11]);
    imu->gyro_z_raw = (int16_t)((data[12] << 8) | data[13]);
    /* Convert Accelerometer Raw -> g */
    imu->acc_x = (float)imu->acc_x_raw / ACCELERATION_RATE;
    imu->acc_y = (float)imu->acc_y_raw / ACCELERATION_RATE;
    imu->acc_z = (float)imu->acc_z_raw / ACCELERATION_RATE;

    imu->acc_x = imu->acc_x - imu->acc_x_bias;
    imu->acc_y = imu->acc_y - imu->acc_y_bias;
    imu->acc_z = imu->acc_z - imu->acc_z_bias;

    imu->temperature = ((float)imu->temperature_raw / 340.0f) + (float)36.53f;

    /* Convert Gyroscope Raw -> degree/sec */
    imu->gyro_x = (((float)imu->gyro_x_raw) - ((float)imu->roll_calibration_value)) / GYRO_RATE;
    imu->gyro_y = (((float)imu->gyro_y_raw) - ((float)imu->pitch_calibration_value))/ GYRO_RATE;
    imu->gyro_z = (((float)imu->gyro_z_raw) - ((float)imu->yaw_calibration_value))/ GYRO_RATE;

    //printf("Acc_x:%.2f,Acc_y:%.2f,Acc_z:%.2f  Gyro_x:%.2f,Gyro_y:%.2f,Gyro_z:%.2f\r\n",mpu6050->acc_x, mpu6050->acc_y, mpu6050->acc_z,
    //		mpu6050->gyro_x, mpu6050->gyro_y, mpu6050->gyro_z);
}











