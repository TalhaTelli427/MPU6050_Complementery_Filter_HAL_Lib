# MPU6050_Complementery_Filter_HAL_Lib For Novohead 

The aim of the project is to create an algorithm that detects the body position of the user with the help of the sensor and microcontroller placed on the VR headset. I used STM32F446RE microcontroller and MPU6050 IMU (Gyroscope and Acceleration sensor) to make this happen. I also provided the communication between MPU6050 and STM32 with I2C communication protocol.
## Block Diagram

![Novohead Diyagram drawio](https://github.com/TalhaTelli427/MPU6050_Complementery_Filter_HAL_Lib/assets/132828233/1b65f850-68d5-4ec4-8687-abf15fc8f47e)



## Circuit Image


![Circuit Image](https://github.com/TalhaTelli427/MPU6050_Complementery_Filter_HAL_Lib/assets/132828233/50dd8640-d02e-40e5-9ef6-1fa706b3506a)

  
## Test Video

![ezgif-2-8768fdec2f](https://github.com/TalhaTelli427/MPU6050_Complementery_Filter_HAL_Lib/assets/132828233/13252d63-261e-4956-8d02-f9a1ccf2a38d)

  
  
## Detailed Description Of The Code

First, we perform the initialization settings required to use the MPU6050 sensor.
```C
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

```

Then we read 2000 gyro values ​​and divide it by 2000 and after waiting 1000 ms, we turn on the green LED on the STM32. In this way, the user is informed that the system is ready for operation. The reason why we read 2000 gyro values ​​is to minimize the drift problem of the gyroscope sensor.
```C
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
```

After the calibration data is obtained, the system makes a gyro reading and subtracts the calibration value from the raw gyro value read, thus minimizing the dirft problem. And the  given in the datasheet is divided by the LSB sensitivity parameter.
```C
void TT_Get_All_Values(void) {

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
```

After taking the gyro value, the accelerometer values ​​are read and the read value is divided by the LSB sensitivity parameter given in the datasheet.

```C
HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADRR, MPU6050_ACCE_MESURE, 1,
		Acce_Buffer, 6, 100000);

Acce_Raw_X = (Acce_Buffer[0] << 8 | Acce_Buffer[1]);
Acce_Raw_Y = (Acce_Buffer[2] << 8 | Acce_Buffer[3]);
Acce_Raw_Z = (Acce_Buffer[4] << 8 | Acce_Buffer[5]);

Acce_X = (float) Acce_Raw_X / 4096;
Acce_Y = (float) Acce_Raw_Y / 4096;
Acce_Z = (float) Acce_Raw_Z / 4096;

```

Using the acceleration values ​​obtained, the angle of the x and y axes to the ground is calculated. The obtained pitch and roll values ​​are passed through a low-pass filter. The pitch and roll angle values ​​calculate the tilt of the user's head to the ground. This allows the user to look up and down and down in our algorithm. The reason why we don't do it with an accelerometer is that the accelerometer sensor is very easily affected by noise and cannot calculate the rotations in the yaw axis. I used the low-pass filter to minimize the noise as much as possible.

```C
    // Acce Angel Calculation
Mesured_Angle_Pitch_Acce = (atan(Acce_Y / sqrt(Acce_X * Acce_X + Acce_Z * Acce_Z)) * 1
			/ (3.142 / 180) + 1);
Mesured_Angle_Roll_Acce = (-atan(Acce_X / sqrt(Acce_Y * Acce_Y + Acce_Z * Acce_Z)) * 1
			/ (3.142 / 180) - 0.80);

New_Angel_Pitch_Acce=Old_Angle_Pitch_Acce*0.7+Mesured_Angle_Pitch_Acce*0.3;
New_Angel_Roll_Acce=Old_Angle_Roll_Acce*0.7+Mesured_Angle_Roll_Acce*0.3;
Old_Angle_Pitch_Acce=New_Angel_Pitch_Acce;
Old_Angle_Roll_Acce=New_Angel_Roll_Acce;
  ```

Even if we calibrate and filter the gyro and acceleration values, it is not enough. Accelerometer and gyros sensor have their own advantages and disadvantages. While the gyro sensor gives accurate results in the short term, it starts to make mistakes in the long term due to the drift problem. While the accelerometer makes accurate measurements in the long term, it is more affected by the noise in the short term and returns you an incorrect value. Complementary filter allows us to obtain the most accurate angle value by using both sensors.

```C
// Complementery filter
Filterd_Pitch = (0.96f) * (Filterd_Pitch + Gyro_X * dt)
		+ (New_Angel_Pitch_Acce * 0.04f);
Filterd_Roll = (0.96f) * (Filterd_Roll + Gyro_Y * dt)
		+ (New_Angel_Roll_Acce *0.04f);
```
Since there is no magnetometer on the MPU6050 to filter and measure the yaw axis, it is passed through a high-pass filter and used that way.
```C
Angle_Yaw_Gyro += Gyro_Z * dt * 0.98039215686274;

```

Since there is no magnetometer on the MPU6050 to filter and measure the yaw axis, it is passed through a high-pass filter and used that way.
```C
//This process minimizes the noise applied by the axes to each other.
Filterd_Pitch += Filterd_Roll * sin(Gyro_Z * Omega);
Filterd_Roll -= Filterd_Pitch * sin(Gyro_Z * Omega);

```
# Conclusion

As a result, with an IMU sensor on the VR headset, we can track the person's head up and down position, tilting position to the right and left, and finally the position of turning the head to the left and right.


If you want to go to the   [Soruce](TT_MPU6050_446_RE_LIB/Core/Src/TT_mpu6050.c).
If you want to go to the   [Header](TT_MPU6050_446_RE_LIB/Core/Inc/TT_MPU6050.h).
If you want to go to the   [Main](TT_MPU6050_446_RE_LIB/Core/Src/main.c).

