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
int16_t Gyro_Cal_Raw_X,Gyro_Cal_Raw_Y,Gyro_Cal_Raw_Z;
int16_t Gyro_Raw_X, Gyro_Raw_Y,Gyro_Raw_Z;
int16_t Acce_Raw_X,Acce_Raw_Y,Acce_Raw_Z;
int16_t Acce_Raw_Cal_X,Acce_Raw_Cal_Y,Acce_Raw_Cal_Z;
float Gyro_X,Gyro_Y,Gyro_Z;
float Acce_X,Acce_Y,Acce_Z;
float Gyro_Cal_X,Gyro_Cal_Y,Gyro_Cal_Z;
float angle_pitch_gyro,angle_roll_gyro,angle_yaw_gyro;
float Angle_Pitch_Acce,Angle_Roll_Acce;
float Filterd_Pitch,Filterd_Roll;
uint32_t Time;

void TT_Init_MPU6050(void){

	  data[0]=0x00;
	  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADRR, MPU6050_POWER_1,1,data, 1, 10000);
	  data[0]=0x02;
	  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADRR, MPU6050_CONFIG,1,data, 1, 10000);
	  data[0]=0x10;
	  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADRR, MPU6050_ACCE_CONFIG,1,data, 1, 10000);
	  data[0]=0x08;
	  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADRR, MPU6050_GYRO_CONFIG,1,data, 1, 10000);
	}
void TT_Get_Gyro_Cal_Values(void){

	  for( int i=0;i<4000;i++){

		  HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADRR,MPU6050_GYRO_MESURE,1,Gyro_Cal_Buffer,6,HAL_MAX_DELAY);
		  Gyro_Cal_Raw_X=(Gyro_Cal_Buffer[0]<<8 | Gyro_Cal_Buffer[1]);
		  Gyro_Cal_Raw_Y=(Gyro_Cal_Buffer[2]<<8 | Gyro_Cal_Buffer[3]);
		  Gyro_Cal_Raw_Z=(Gyro_Cal_Buffer[4]<<8 | Gyro_Cal_Buffer[5]);

		  Gyro_Cal_X+=(float)Gyro_Cal_Raw_X;
		  Gyro_Cal_Y+=(float)Gyro_Cal_Raw_Y;
		  Gyro_Cal_Z+=(float)Gyro_Cal_Raw_Z;

		  HAL_Delay(10);

	  }

	  Gyro_Cal_X/=4000;
	  Gyro_Cal_Y/=4000;
	  Gyro_Cal_Z/=4000;
	  HAL_Delay(1000);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,SET);

}

void TT_Get_All_Values(void){

	 if(HAL_GetTick()-Time>=20){

		  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADRR,MPU6050_GYRO_MESURE,1,Gyro_Buffer, 6, 100000);

		  Gyro_Raw_X=(Gyro_Buffer[0]<<8 | Gyro_Buffer[1]);
		  Gyro_Raw_Y=(Gyro_Buffer[2]<<8 | Gyro_Buffer[3]);
		  Gyro_Raw_Z=(Gyro_Buffer[4]<<8 | Gyro_Buffer[5]);

		 Gyro_Raw_X-=Gyro_Cal_X;
		 Gyro_Raw_Y-=Gyro_Cal_Y;
		 Gyro_Raw_Z-=Gyro_Cal_Z;

		 Gyro_X=Gyro_Raw_X /65.5;
		 Gyro_Y=Gyro_Raw_Y /65.5;
		 Gyro_Z=Gyro_Raw_Z /65.5;

		 HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADRR,MPU6050_ACCE_MESURE,1,Acce_Buffer, 6, 100000);

		 	  Acce_Raw_X=(Acce_Buffer[0]<<8 | Acce_Buffer[1]);
		 	  Acce_Raw_Y=(Acce_Buffer[2]<<8 | Acce_Buffer[3]);
		 	  Acce_Raw_Z=(Acce_Buffer[4]<<8 | Acce_Buffer[5]);

		 	  Acce_X=(float)Acce_Raw_X/4096;
		 	  Acce_Y=(float)Acce_Raw_Y/4096;
		 	  Acce_Z=(float)Acce_Raw_Z/4096;

		 	  Angle_Pitch_Acce=(atan(Acce_Y / sqrt(Acce_X * Acce_X + Acce_Z * Acce_Z)) * 1 / (3.142 / 180)+1);
		 	  Angle_Roll_Acce=(-atan(Acce_X/sqrt(Acce_Y*Acce_Y+Acce_Z*Acce_Z))*1/(3.142/180)-0.80);

		 	  Filterd_Pitch =  (0.96f)*(Filterd_Pitch+Gyro_X*0.02f)+ (Angle_Pitch_Acce * 0.04f);
		 	  Filterd_Roll =  (0.96f) * (Filterd_Roll+Gyro_Y*0.02f ) + (Angle_Roll_Acce * 0.04f);

		 	  angle_yaw_gyro+=Gyro_Z*0.02f* 0.98039215686274;
		 	  Filterd_Pitch += Filterd_Roll * sin(Gyro_Raw_Z * 0.00000532924);
		 	  Filterd_Roll -= Filterd_Pitch * sin(Gyro_Raw_Z * 0.00000532924);
		 	  Time = HAL_GetTick();
	 }
	 }
float TT_Get_Pitch_Angle(void){

	TT_Get_All_Values();
	return Filterd_Pitch;
}
float TT_Get_Roll_Angle(void){

	TT_Get_All_Values();
	return Filterd_Roll;
}
float TT_Get_Yaw_Angle(void){

	TT_Get_All_Values();
	return angle_yaw_gyro;
}
