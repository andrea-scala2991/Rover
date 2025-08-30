/*
    Copyright (C) 2025  Andrea

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

TRACCIA: 
Progettare e implementare un rover in grado di eseguire i comandi ricevuti tramite un controller il rover può trovarsi in tre diversi stati operativi: OK quando il sistema funziona correttamente senza anomalie, DEGRADATO in caso di malfunzionamento parziale che riduce alcune funzionalità ma mantiene il rover operativo, EMERGENZA quando il rover entra in uno stato di sicurezza per prevenire danni limitando o sospendendo le operazioni.
*/

#include "MPU-6050.h"

void MPU6050_config(I2C_HandleTypeDef* hi2c){
  uint8_t check = 0;
  uint8_t Data;

  HAL_I2C_Mem_Read(hi2c, MPU6050_ADDRESS, 0x75, 1, &check, 1, 1000);  // read WHO_AM_I
  if (check == 0x68){  // 0x68 will be returned by the sensor if everything goes well
	  // power management register 0X6B we should write all 0's to wake the sensor up
	  Data = 0;
	  HAL_I2C_Mem_Write(hi2c, MPU6050_ADDRESS, 0x6B, 1, &Data, 1, 1000);

	  // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
	  Data = 0x07;
	  HAL_I2C_Mem_Write(hi2c, MPU6050_ADDRESS, 0x19, 1, &Data, 1, 1000);
	  // Set accelerometer configuration in ACCEL_CONFIG Register
	  Data = 0x00;  // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
	  HAL_I2C_Mem_Write(hi2c, MPU6050_ADDRESS, 0x1C, 1, &Data, 1, 1000);

	  // Set Gyroscopic configuration in GYRO_CONFIG Register
	  Data = 0x00;  // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 ̐/s
	  HAL_I2C_Mem_Write(hi2c, MPU6050_ADDRESS, 0x1B, 1, &Data, 1, 1000);
	  }
}

int MPU6050_Read_Accel(accelerometer_t* accelerometer, I2C_HandleTypeDef* hi2c)
{
	uint8_t acc_data[6];
	int16_t accel_X_RAW, accel_Y_RAW, accel_Z_RAW;

	// Read 6 BYTES of data starting from ACCEL_XOUT_H (0x3B) register
	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDRESS, 0x3B, 1, acc_data, 6, 1000) != HAL_OK)
		return -1;

	accel_X_RAW = (int16_t)(acc_data[0] << 8 | acc_data [1]);
	accel_Y_RAW = (int16_t)(acc_data[2] << 8 | acc_data [3]);
	accel_Z_RAW = (int16_t)(acc_data[4] << 8 | acc_data [5]);

	//get acceleration in g's
	accelerometer->acc_x = (float)accel_X_RAW/16384.0;
	accelerometer->acc_y = (float)accel_Y_RAW/16384.0;
	accelerometer->acc_z = (float)accel_Z_RAW/16384.0;
	return 0;
}

void MPU6050_Read_Gyro(accelerometer_t* accelerometer, I2C_HandleTypeDef* hi2c)
{
	uint8_t acc_data[6];
	int16_t gyro_X_RAW, gyro_Y_RAW, gyro_Z_RAW;

	// Read 6 BYTES of data starting from GYRO_XOUT_H (0x43) register
	HAL_I2C_Mem_Read(hi2c, MPU6050_ADDRESS, 0x43, 1, acc_data, 6, 1000);
	gyro_X_RAW = (int16_t)(acc_data[0] << 8 | acc_data [1]);
	gyro_Y_RAW = (int16_t)(acc_data[2] << 8 | acc_data [3]);
	gyro_Z_RAW = (int16_t)(acc_data[4] << 8 | acc_data [5]);

	//get gyroscope values in degrees per second
	accelerometer->acc_x = (float)gyro_X_RAW/131.0;
	accelerometer->acc_y = (float)gyro_Y_RAW/131.0;
	accelerometer->acc_z = (float)gyro_Z_RAW/131.0;
}
