/*
 * TT_mpu6050.c
 *
 *  Created on: May 26, 2023
 *      Author: talha
 */
#include "TT_MPU6050.h"

uint8_t data[1];
uint8_t Gyro_Buffer[6];
uint8_t Gyro_Cal_Buffer[6];
uint8_t Acce_Buffer[6];
int16_t Gyro_Cal_Raw_X, Gyro_Cal_Raw_Y, Gyro_Cal_Raw_Z;
int16_t Gyro_Raw_X, Gyro_Raw_Y, Gyro_Raw_Z;
int16_t Acce_Raw_X, Acce_Raw_Y, Acce_Raw_Z;
int16_t Acce_Raw_Cal_X, Acce_Raw_Cal_Y, Acce_Raw_Cal_Z;
float Gyro_X, Gyro_Y, Gyro_Z;
float Acce_X, Acce_Y, Acce_Z;
float Gyro_Cal_X, Gyro_Cal_Y, Gyro_Cal_Z;
float Angle_Yaw_Gyro;
float Mesured_Angle_Pitch_Acce,Old_Angle_Pitch_Acce,New_Angel_Pitch_Acce ;
float Mesured_Angle_Roll_Acce,Old_Angle_Roll_Acce,New_Angel_Roll_Acce;
float Filterd_Pitch, Filterd_Roll,Filterd_Yaw;
const float dt=0.02f,Radyan_to_Degree=57.2957795131f
		,Omega=0.00034906585;// Omega= dt/Radyan_to_Degree
uint32_t Time=0;




void TT_Init_MPU6050(void) {
	// Power Conifg
	data[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADRR, MPU6050_POWER_1, 1, data, 1, 10000);
	// Internal Low Pass Filter Config
	data[0] = 0x04;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADRR, MPU6050_CONFIG, 1, data, 1, 10000);
	//Acceloremter Config +-4g
	data[0] = 0x10;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADRR, MPU6050_ACCE_CONFIG, 1, data, 1,
			10000);
	// Gyro Config ± 500 °/s
	data[0] = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADRR, MPU6050_GYRO_CONFIG, 1, data, 1,
			10000);
}

void TT_Get_Gyro_Cal_Values(void) {

	for (int i = 0; i < 2000; i++) {

		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADRR, MPU6050_GYRO_MESURE, 1,
				Gyro_Cal_Buffer, 6, HAL_MAX_DELAY);

		Gyro_Cal_Raw_X = (Gyro_Cal_Buffer[0] << 8 | Gyro_Cal_Buffer[1]);
		Gyro_Cal_Raw_Y = (Gyro_Cal_Buffer[2] << 8 | Gyro_Cal_Buffer[3]);
		Gyro_Cal_Raw_Z = (Gyro_Cal_Buffer[4] << 8 | Gyro_Cal_Buffer[5]);

		Gyro_Cal_X += (float) Gyro_Cal_Raw_X;
		Gyro_Cal_Y += (float) Gyro_Cal_Raw_Y;
		Gyro_Cal_Z += (float) Gyro_Cal_Raw_Z;

		HAL_Delay(20);

	}

	Gyro_Cal_X /= 2000;
	Gyro_Cal_Y /= 2000;
	Gyro_Cal_Z /= 2000;
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);

}

void TT_Get_All_Values(void) {

	//For the gyro to measure accurately, operation is performed every 20 ms or greater
	if (HAL_GetTick() - Time >= 20) {
		//Getting Gyro data
		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADRR, MPU6050_GYRO_MESURE, 1,
				Gyro_Buffer, 6, 100000);

		Gyro_Raw_X = (Gyro_Buffer[0] << 8 | Gyro_Buffer[1]);
		Gyro_Raw_Y = (Gyro_Buffer[2] << 8 | Gyro_Buffer[3]);
		Gyro_Raw_Z = (Gyro_Buffer[4] << 8 | Gyro_Buffer[5]);

		Gyro_Raw_X -= Gyro_Cal_X;
		Gyro_Raw_Y -= Gyro_Cal_Y;
		Gyro_Raw_Z -= Gyro_Cal_Z;

		Gyro_X = Gyro_Raw_X / 65.5;
		Gyro_Y = Gyro_Raw_Y / 65.5;
		Gyro_Z = Gyro_Raw_Z / 65.5;
		//Getting Acce Data
		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADRR, MPU6050_ACCE_MESURE, 1,
				Acce_Buffer, 6, 100000);

		Acce_Raw_X = (Acce_Buffer[0] << 8 | Acce_Buffer[1]);
		Acce_Raw_Y = (Acce_Buffer[2] << 8 | Acce_Buffer[3]);
		Acce_Raw_Z = (Acce_Buffer[4] << 8 | Acce_Buffer[5]);

		Acce_X = (float) Acce_Raw_X / 4096;
		Acce_Y = (float) Acce_Raw_Y / 4096;
		Acce_Z = (float) Acce_Raw_Z / 4096;
		// Acce Angel Calculation
		Mesured_Angle_Pitch_Acce = (atan(
				Acce_Y / sqrt(Acce_X * Acce_X + Acce_Z * Acce_Z)) * 1
				/ (3.142 / 180) + 1);
		Mesured_Angle_Roll_Acce = (-atan(
				Acce_X / sqrt(Acce_Y * Acce_Y + Acce_Z * Acce_Z)) * 1
				/ (3.142 / 180) - 0.80);

		New_Angel_Pitch_Acce=Old_Angle_Pitch_Acce*0.7+Mesured_Angle_Pitch_Acce*0.3;
		New_Angel_Roll_Acce=Old_Angle_Roll_Acce*0.7+Mesured_Angle_Roll_Acce*0.3;
		Old_Angle_Pitch_Acce=New_Angel_Pitch_Acce;
		Old_Angle_Roll_Acce=New_Angel_Roll_Acce;


		// Complementery filter
		Filterd_Pitch = (0.96f) * (Filterd_Pitch + Gyro_X * dt)
				+ (New_Angel_Pitch_Acce * 0.04f);
		Filterd_Roll = (0.96f) * (Filterd_Roll + Gyro_Y * dt)
				+ (New_Angel_Roll_Acce *0.04f);

		//High Pass filter for yaw angle
		Angle_Yaw_Gyro += Gyro_Z * dt * 0.98039215686274;

		//This process minimizes the noise applied by the axes to each other.
		Filterd_Pitch += Filterd_Roll * sin(Gyro_Z * 0.00034906585);
		Filterd_Roll -= Filterd_Pitch * sin(Gyro_Z * 0.00034906585);
	//	Angle_Yaw_Gyro -= Filterd_Roll * sin(Gyro_Raw_Z * 0.00000532924);

		Time = HAL_GetTick();
	}
}
void TT_UART_Data_Transmitter(float Transmit_Data){
	char tampon[8];

	sprintf(tampon,"%.2f",Transmit_Data);
	HAL_UART_Transmit(&huart2, (uint8_t*)tampon, sizeof(tampon), HAL_MAX_DELAY);

}

float TT_Get_Pitch_Angle(void) {

	TT_Get_All_Values();
	return Filterd_Pitch;
}
float TT_Get_Roll_Angle(void) {

	TT_Get_All_Values();
	return Filterd_Roll;
}
float TT_Get_Yaw_Angle(void) {

	TT_Get_All_Values();
	return Angle_Yaw_Gyro;
}
