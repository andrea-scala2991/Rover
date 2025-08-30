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


#ifndef INC_MPU_6050_H_
#define INC_MPU_6050_H_

#include "i2c.h"
//assuming the AD0 pin is set to low
#define MPU6050_ADDRESS	0x68 << 1

typedef struct {
	int16_t		acc_x;			//accelerometer value of x axis
	int16_t		acc_y;			//accelerometer value of y axis
	int16_t		acc_z;			//accelerometer value of z axis
	int16_t		gyro_x;			//gyroscope value of x axis
	int16_t		gyro_y;			//gyroscope value of x axis
	int16_t		gyro_z;			//gyroscope value of x axis
} accelerometer_t;

void MPU6050_config(I2C_HandleTypeDef* hi2c);
int MPU6050_Read_Accel(accelerometer_t* accelerometer, I2C_HandleTypeDef* hi2c);
void MPU6050_Read_Gyro(accelerometer_t* accelerometer, I2C_HandleTypeDef* hi2c);

#endif /* INC_MPU_6050_H_ */
