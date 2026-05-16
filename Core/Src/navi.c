#include "navi.h"
#include "cmsis_os.h"
#include "motor.h"
#include "mpu6050.h"
#include <stdlib.h>


float angle_ratio = 1.05f; // 航向角补偿系数
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
  if (gyro_z > -0.5f && gyro_z < 0.5f)
  {
    gyro_z = 0.0f;
  }
  robot_yaw_angle += (gyro_z * 0.01f * angle_ratio);

  // 2. 只有在直线模式下，且基础速度不为0时，才进行纠偏
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
      int right_boost = 3;

      // 纠偏逻辑加上物理补偿
      pid_left.target = base_speed - correction;
      pid_right.target = base_speed + correction + right_boost;
    }
  }

  // 3. 计算底层 PID
  int16_t current_L = Read_Left_Encoder();
  int16_t current_R = Read_Right_Encoder();

  // 里程计功能：只要轮子在转，无论前进后退都累加绝对值
  global_mileage_L += abs(current_L);
  global_mileage_R += abs(current_R);

  int out_L = (int)PID_Calc(&pid_left, current_L);
  int out_R = (int)PID_Calc(&pid_right, current_R);
  Motor_SetSpeed(out_L, out_R);
}

// 精准转弯函数
void Navi_Turn_Angle(float turn_angle)
{
  float target_a = robot_yaw_angle + turn_angle;
  uint32_t start_time = HAL_GetTick();

  base_speed = 0;
  navi_mode = 1; // 告诉后台：现在是转弯模式，别管直线纠偏了

  while (1)
  {
    // ==========================================
    // 不要去调 Navi_Update，直接去延时 10ms！Task_Control 会在后台帮你调
    // Navi_Update 驱动轮子
    // ==========================================
    osDelay(10);

    float error = target_a - robot_yaw_angle;

    // 角度到位，跳出
    if (error > -5.0f && error < 5.0f)
    {
      break;
    }

    // 超时保护，跳出
    if (HAL_GetTick() - start_time > 3000)
    {
      break;
    }

    // 计算当前需要的转速
    int turn_speed = (int)(error * 1.8f);

    if (turn_speed > 0 && turn_speed < 18)
      turn_speed = 18;
    if (turn_speed < 0 && turn_speed > -18)
      turn_speed = -18;

    if (turn_speed > 45)
      turn_speed = 45;
    if (turn_speed < -45)
      turn_speed = -45;

    // 把算好的速度喂给 PID 目标，后台的 Navi_Update() 会自动执行它！
    pid_left.target = -turn_speed;
    pid_right.target = turn_speed;
  }

  // 刹车停稳
  pid_left.target = 0;
  pid_right.target = 0;
  navi_mode = 0;

  // 把裸机的 HAL_Delay 换成操作系统的 osDelay
  osDelay(200);

  // 更新直线行驶的目标航向
  target_yaw = robot_yaw_angle;
}

// ==========================================
// 紧急逃生后退函数（基于时间，但保持 PID 闭环心跳）
// ==========================================
void Navi_Escape_Backward(int speed, uint32_t ms_time)
{
  // 1. 下达直线后退命令，后台任务会瞬间接管，自动走直线
  Navi_Set_Straight(speed);

  // 2.  延时ms_time 这么长的时间
  osDelay(ms_time);

  // 3. 把速度设为 0，逃生结束
  Navi_Set_Straight(0);
  pid_left.target = 0;
  pid_right.target = 0;
}
