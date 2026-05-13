#ifndef __NAVI_H
#define __NAVI_H

#include "main.h"

// 声明一个全局标志位，用于 10ms 节拍通信
extern volatile uint8_t flag_10ms;
extern float robot_yaw_angle; // 暴露给外部查看当前角度

extern int32_t global_mileage_L; // 左轮累计里程
extern int32_t global_mileage_R; // 右轮累计里程

// 导航系统接口
void Navi_Init(void);
void Navi_Update(void);              // 10ms 核心刷新函数
void Navi_Set_Straight(int speed);   // 设定直线巡航速度
void Navi_Turn_Angle(float angle);   // 精准转弯函数
void Navi_Escape_Backward(int speed, uint32_t ms_time); //逃生函数

#endif /* __NAVI_H */
