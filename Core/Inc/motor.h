#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

// 声明在 main.c 中或者 stm32f1xx_hal_conf.h 中定义的定时器句柄
extern TIM_HandleTypeDef htim1; // 右轮编码器 TIM1
extern TIM_HandleTypeDef htim2; // 电机 PWM TIM2
extern TIM_HandleTypeDef htim4; // 左轮编码器 TIM4 

// ==========================================
// 1. PID 结构体定义 (重点新增)
// ==========================================
typedef struct {
    float kp;          // 比例系数
    float ki;          // 积分系数
    float kd;          // 微分系数
    float target;      // 目标速度 (期望编码器脉冲数)
    float actual;      // 真实速度 (当前编码器脉冲数)
    float error;       // 当前偏差
    float last_error;  // 上次偏差
    float integral;    // 积分累计值
    float pwm_out;     // 计算后输出的 PWM 值
} PID_TypeDef;

// ==========================================
// 2. 外部调用的全局 PID 变量声明 (重点新增)
// ==========================================
extern PID_TypeDef pid_left;
extern PID_TypeDef pid_right;

// ==========================================
// 3. 原有的函数声明
// ==========================================
void Motor_Init(void);
void Motor_SetSpeed(int left_speed, int right_speed);

void Robot_MoveForward(int speed);
void Robot_MoveBackward(int speed);
void Robot_SpinLeft(int speed);
void Robot_SpinRight(int speed);
void Robot_Stop(void);

// ==========================================
// 4. 新增的闭环控制函数声明
// ==========================================
int16_t Read_Left_Encoder(void);
int16_t Read_Right_Encoder(void);
float PID_Calc(PID_TypeDef *pid, float current_speed);

#endif /* __MOTOR_H */
