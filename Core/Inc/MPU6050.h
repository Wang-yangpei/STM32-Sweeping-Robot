#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

// MPU6050 I2C 地址 (AD0接GND时为0xD0)
#define MPU6050_ADDR 0xD0

extern I2C_HandleTypeDef hi2c1; // 声明 CubeMX 生成的 I2C 句柄

void MPU6050_Init(void);
void MPU6050_Read_Gyro(float *gyro_z);
void MPU6050_Calibrate(void);

#endif
