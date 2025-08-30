/*
 * MPU-6050.h
 *
 *  Created on: Dec 22, 2024
 *      Author: andrea
 *
 * library for accelerometer and gyroscope IC MPU-6050
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
