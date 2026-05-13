#include "motor.h"

// 初始化：开启 PWM 信号输出
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // 左轮 PA3
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 右轮 PA0

   
}

// 核心调速函数 (左轮A通道，右轮B通道)
void Motor_SetSpeed(int left_speed, int right_speed)
{
    // 1. 控制左轮 (A通道：PA1, PA2)
    if (left_speed >= 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, left_speed); // 设置占空比
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, -left_speed); // PWM 只能接收正数
    }

    // 2. 控制右轮 (B通道：PB0, PB1)
    if (right_speed >= 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, right_speed);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -right_speed);
    }
}

// === 机器人动作库实现 ===
void Robot_MoveForward(int speed)
{
    Motor_SetSpeed(speed, speed);
}

void Robot_MoveBackward(int speed)
{
    Motor_SetSpeed(-speed, -speed);
}

// 左转：左轮倒退，右轮前进
void Robot_SpinLeft(int speed)
{
    Motor_SetSpeed(-speed, speed);
}

// 右转：左轮前进，右轮倒退
void Robot_SpinRight(int speed)
{
    Motor_SetSpeed(speed, -speed);
}

void Robot_Stop(void)
{
    Motor_SetSpeed(0, 0);
}

// ==========================================
// ！！！以下是新增的闭环 PID 与编码器代码 ！！！
// ==========================================

// 1. 初始化左右轮的 PID 参数 (Kp, Ki, Kd 需要根据实际小车微调)
PID_TypeDef pid_left  = {8.0f, 0.5f, 0.1f, 0, 0, 0, 0, 0, 0};
PID_TypeDef pid_right = {8.0f, 0.5f, 0.1f, 0, 0, 0, 0, 0, 0};

// 2. 读取左轮真实速度 (连接在 TIM4)
int16_t Read_Left_Encoder(void)
{
    // 读取计数器并强制转换为有符号数
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0); // 读完立刻清零
    return count;
}

// 3. 读取右轮真实速度 (连接在 TIM1)
int16_t Read_Right_Encoder(void)
{
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    return -count;
}

// 4. PID 计算核心函数
float PID_Calc(PID_TypeDef *pid, float current_speed)
{
    // 更新当前速度和计算偏差
    pid->actual = current_speed;

    // ==========================================
    // 【核心修复】：停车斩断！
    // 如果目标速度是 0，说明我们要彻底刹车。
    // 必须清空过去累积的积分和误差，直接输出 0！
    // ==========================================
    if (pid->target == 0)
    {
        pid->error = 0;
        pid->last_error = 0;
        pid->integral = 0; // 彻底清空记忆！
        pid->pwm_out = 0;
        return 0.0f;       // 彻底切断 PWM 输出
    }

    pid->error = pid->target - pid->actual;

    // 累计积分，并进行积分限幅 (防止持续堵转时积分过大，俗称抗积分饱和)
    pid->integral += pid->error;
    if (pid->integral > 2000.0f)
        pid->integral = 2000.0f;
    if (pid->integral < -2000.0f)
        pid->integral = -2000.0f;

    // 核心公式计算
    pid->pwm_out = (pid->kp * pid->error) +
                   (pid->ki * pid->integral) +
                   (pid->kd * (pid->error - pid->last_error));

    // 保存偏差留给下次算微分
    pid->last_error = pid->error;

    // PWM 输出限幅 (假设你的 TIM2 AutoReload 设为了 1000)
    // 根据你的实际 ARR 值来定，这里假设最大 PWM 是 1000
    if (pid->pwm_out > 1000.0f)
        pid->pwm_out = 1000.0f;
    if (pid->pwm_out < -1000.0f)
        pid->pwm_out = -1000.0f;

    return pid->pwm_out;
}
