/*
 * TT_MPU6050.h
 *
 *  Created on: May 26, 2023
 *      Author: talha
 */

#ifndef INC_TT_MPU6050_H_
#define INC_TT_MPU6050_H_
#include "stm32f4xx_hal.h"
#include <math.h>

#define MPU6050_ADRR 0X68<<1
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCE_CONFIG 0x1C
#define MPU6050_ACCE_MESURE 0x3B
#define MPU6050_GYRO_MESURE 0x43
#define MPU6050_POWER_1 0x6B
#define MPU6050_CONFIG 0x1A

extern I2C_HandleTypeDef hi2c1;

void TT_Init_MPU6050(void);
void TT_Get_Gyro_Cal_Values(void);
void TT_Get_All_Values(void);
float TT_Get_Pitch_Angle(void);
float TT_Get_Roll_Angle(void);
float TT_Get_Yaw_Angle(void);
#endif /* INC_TT_MPU6050_H_ */
