#include "mpu6050.h"

// 寄存器宏定义
#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define GYRO_ZOUT_H  0x47

float gyro_z_offset = 0.0f; // 用于存放 Z 轴的静态零漂值

// 封装一个简单的写寄存器函数
void MPU6050_WriteByte(uint8_t reg, uint8_t data)
{
    // HAL 库自带的超级函数，直接搞定发送和等待
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

// MPU6050 初始化
void MPU6050_Init(void)
{
    MPU6050_WriteByte(PWR_MGMT_1, 0x00);  // 解除休眠状态
    MPU6050_WriteByte(SMPLRT_DIV, 0x07);  // 采样率分频，1kHz
    MPU6050_WriteByte(CONFIG, 0x06);      // 低通滤波
    MPU6050_WriteByte(GYRO_CONFIG, 0x18); // 陀螺仪量程：±2000°/s
}

// 陀螺仪静态零偏校准函数
void MPU6050_Calibrate(void)
{
    float sum = 0.0f;
    uint8_t data[2];
    int16_t raw_z;
    float temp_z;
    
    // 读 200 次数据求平均，大约需要 1 秒钟
    for(int i = 0; i < 200; i++)
    {
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_ZOUT_H, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
        raw_z = (data[0] << 8) | data[1];
        temp_z = (float)raw_z / 16.4f;
        sum += temp_z;
        
        HAL_Delay(5); // 每次读完稍微等一下
    }
    
    // 算出零漂的平均偏差
    gyro_z_offset = sum / 200.0f; 
}

// 读取 Z 轴陀螺仪数据 每次读取都要减去刚才算出来的零漂！ (扫地机主要看水平面旋转，只读 Z 轴即可)
void MPU6050_Read_Gyro(float *gyro_z)
{
    uint8_t data[2];
    int16_t raw_z;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_ZOUT_H, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
    raw_z = (data[0] << 8) | data[1];
    
    // 获取原始角速度，并强行减去静止时的偏差
    float actual_z = ((float)raw_z / 16.4f) - gyro_z_offset; 
    
    *gyro_z = actual_z; 
}
