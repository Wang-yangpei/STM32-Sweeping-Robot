#include "navi.h"
#include "mpu6050.h"
#include "motor.h"
#include <stdlib.h>

volatile uint8_t flag_10ms = 0;
float angle_ratio = 1.05f;//航向角补偿系数
float robot_yaw_angle = 0.0f; 
float target_yaw = 0.0f;      
float Kp_yaw = 2.5f;          
int base_speed = 0; 

int32_t global_mileage_L = 0;
int32_t global_mileage_R = 0;

// 【新增】驾驶模式标志：0代表巡航走直线，1代表原地转弯
uint8_t navi_mode = 0; 

void Navi_Init(void)
{
    robot_yaw_angle = 0.0f;
    target_yaw = 0.0f;
    base_speed = 0;
    navi_mode = 0;
}

void Navi_Set_Straight(int speed)
{
    base_speed = speed;
    target_yaw = robot_yaw_angle; 
    navi_mode = 0; // 切换到直线模式
}

// 核心刷新函数
void Navi_Update(void)
{
    // 1. 读取陀螺仪并积分角度
    float gyro_z = 0;
    MPU6050_Read_Gyro(&gyro_z);
    if(gyro_z > -0.5f && gyro_z < 0.5f) { gyro_z = 0.0f; } 
    robot_yaw_angle += (gyro_z * 0.01f * angle_ratio);

    // 2. 【核心修复】只有在直线模式下，且基础速度不为0时，才进行纠偏
    if (navi_mode == 0)
    {
        if (base_speed == 0) 
        {
            // 如果期望完全停止，就不要做微小的角度纠偏，直接锁死目标为0，彻底消除啸叫
            pid_left.target = 0;
            pid_right.target = 0;
        }
        else 
        {
            float yaw_error = target_yaw - robot_yaw_angle;
            int correction = (int)(yaw_error * Kp_yaw);

            // 往右偏移的底层物理差异补偿值
            // 如果总是往右偏，说明右轮偏弱，给右轮加一点点固定补偿（例如 2~5）
            // 如果某天变成往左偏了，就把这个值改成负数，或者加到左轮上
            int right_boost = 5; 

            // 纠偏逻辑加上物理补偿
            pid_left.target  = base_speed - correction;
            pid_right.target = base_speed + correction + right_boost;
        }
    }

    // 3. 计算底层 PID 
    int16_t current_L = Read_Left_Encoder();
    int16_t current_R = Read_Right_Encoder();

     // 【里程计功能】：只要轮子在转，无论前进后退都累加绝对值
    global_mileage_L += abs(current_L);
    global_mileage_R += abs(current_R);
    
    int out_L = (int)PID_Calc(&pid_left, current_L);
    int out_R = (int)PID_Calc(&pid_right, current_R);
    //Motor_SetSpeed(out_L, out_R);

}

// 精准转弯函数
void Navi_Turn_Angle(float turn_angle)
{
    float target_a = robot_yaw_angle + turn_angle;
    uint32_t start_time = HAL_GetTick(); // 记录开始转弯的时间

    base_speed = 0; 
    navi_mode = 1; 
    
    while (1)
    {
        if (flag_10ms == 1)
        {
            flag_10ms = 0;
            Navi_Update(); 
            
            float error = target_a - robot_yaw_angle;
            
            // 【修改1】稍微放宽窗口到 5 度，提高捕捉成功率
            if (error > -5.0f && error < 5.0f) {
                break; 
            }
            
            // 【修改2】超时保护：转了 3 秒还没对准，强制退出！防止死循环
            if (HAL_GetTick() - start_time > 3000) {
                break;
            }
            
            // 【修改3】平滑降速逻辑
            // 距离目标越近，速度越慢
            int turn_speed = (int)(error * 1.8f); // 比例系数
            
            // 设定死区：转速不能太低，否则转不动；也不能太高，否则冲过头
            // 假设你的电机启动速度是 15
            if (turn_speed > 0 && turn_speed < 18) turn_speed = 18;
            if (turn_speed < 0 && turn_speed > -18) turn_speed = -18;
            
            // 限制最大转速（防止飞出去）
            if (turn_speed > 45) turn_speed = 45;
            if (turn_speed < -45) turn_speed = -45;
            
            pid_left.target = -turn_speed;
            pid_right.target = turn_speed;
        }
    }
    
    // 刹车停稳
    pid_left.target = 0;
    pid_right.target = 0;
    navi_mode = 0; 
    HAL_Delay(200); 
    
    // 更新直线行驶的目标航向
    target_yaw = robot_yaw_angle;
}

// ==========================================
// 紧急逃生后退函数（基于时间，但保持 PID 闭环心跳）
// ==========================================
void Navi_Escape_Backward(int speed, uint32_t ms_time)
{
    // 设定逃生倒车速度
    Navi_Set_Straight(speed); 
    
    uint32_t escape_start = HAL_GetTick();
    
    // 在设定的时间内疯狂倒车
    while (HAL_GetTick() - escape_start < ms_time)
    {
        // 即使在倒车逃跑，也绝不漏掉任何一次 10ms 心跳！
        if (flag_10ms == 1)
        {
            flag_10ms = 0;
            Navi_Update(); 
        }
    }
    
    // 时间到，刹车停稳
    pid_left.target = 0;
    pid_right.target = 0;
}
