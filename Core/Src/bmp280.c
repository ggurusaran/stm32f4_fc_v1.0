/*
 * bmp280.c
 *
 *  Created on: Dec 25, 2024
 *      Author: gurus
 */

#include "bmp280.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

extern I2C_HandleTypeDef hi2c2;

#define SUPPORT_64BIT 1
//#define SUPPORT_32BIT 1


uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;
uint16_t dig_P1;
int16_t  dig_P2;
int16_t  dig_P3;
int16_t  dig_P4;
int16_t  dig_P5;
int16_t  dig_P6;
int16_t  dig_P7;
int16_t  dig_P8;
int16_t  dig_P9;

extern float Temperature;
extern float Pressure;

uint8_t chipID;
int32_t tRaw, pRaw;

extern uint8_t device_id;



void TrimData(void){

	uint8_t rx_buff[24];

	HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDRESS, 0x88, 1, rx_buff, 24, HAL_MAX_DELAY);

	dig_T1=(rx_buff[0])+(rx_buff[1]<<8);
	dig_T2=(rx_buff[2])+(rx_buff[3]<<8);
	dig_T3=(rx_buff[4])+(rx_buff[5]<<8);
	dig_P1=(rx_buff[6])+(rx_buff[7]<<8);
	dig_P2=(rx_buff[8])+(rx_buff[9]<<8);
	dig_P3=(rx_buff[10])+(rx_buff[11]<<8);
	dig_P4=(rx_buff[12])+(rx_buff[13]<<8);
	dig_P5=(rx_buff[14])+(rx_buff[15]<<8);
	dig_P6=(rx_buff[16])+(rx_buff[17]<<8);
	dig_P7=(rx_buff[18])+(rx_buff[19]<<8);
	dig_P8=(rx_buff[20])+(rx_buff[21]<<8);
	dig_P9=(rx_buff[22])+(rx_buff[23]<<8);
}

int BMP280_Config (uint8_t osrs_t, uint8_t osrs_p, uint8_t mode, uint8_t t_sb, uint8_t filter)
{
	TrimData();

	//1,reset the devicee by writing 0xB6
	uint8_t reset_cmd = 0xB6;
	HAL_I2C_Mem_Write(&hi2c2, BMP280_ADDRESS, RESET_REG, 1, &reset_cmd, 1, 1000);
	HAL_Delay(10);

	//2.standbytime, iir filter coeff
	uint8_t config_data = (t_sb<<5)|(filter<<2);
	HAL_I2C_Mem_Write(&hi2c2, BMP280_ADDRESS, CONFIG_REG, 1, &config_data, 1, 1000);
	HAL_Delay(10);

	//3.osrs_T and osrs_P
	uint8_t ctrlmeasdata = (osrs_t<<5)|(osrs_p<<3)|mode;
	HAL_I2C_Mem_Write(&hi2c2, BMP280_ADDRESS, CTRL_MEAS_REG, 1, &ctrlmeasdata, 1, 1000);
	HAL_Delay(10);

	return 0;
}

int BMP280_ReadRaw(void)
{
	uint8_t rawdata[6];

	HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDRESS, ID_REG, 1, &chipID, 1, 1000);

	if(chipID == 0x58){

		//read registers from data registers of bmp280 0xF7 to 0xFC
		HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDRESS, PRESS_MSB_REG, 1, rawdata, 6, 1000);

		pRaw = (rawdata[0]<<12)|(rawdata[1]<<4)|(rawdata[2]>>4);
		tRaw = (rawdata[3]<<12)|(rawdata[4]<<4)|(rawdata[5]>>4);

		return 0;
	}
	else return -1;
}

/*******************************************************************************************************************************
 * Compensation formulae and calculation to find T and P
 */

int32_t t_fine;
int32_t BMP280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1)))>> 12) *((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t BMP280_compensate_P_int64(int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (uint32_t)p;
}

/*
 *******************************************************************************************************************************
 */

void BMP280_Measure (void)
{
	if(BMP280_ReadRaw() == 0){
		if(tRaw == 0x800000) Temperature = 0;
		else
		{
			Temperature = BMP280_compensate_T_int32(tRaw);
		}
		if(pRaw == 0x800000) Pressure = 0;
		else
		{
			Pressure = BMP280_compensate_P_int64(pRaw);
		}
	}
	else{
		Temperature = Pressure = 0; //device is detached
	}
}




