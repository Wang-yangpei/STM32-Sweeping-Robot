/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bumper.h"
#include "cliff.h"
#include "hc_sr04.h"
#include "motor.h"
#include "mpu6050.h"
#include "navi.h"
#include <stdlib.h> // 用于 rand() 随机数

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern uint8_t zigzag_state;
extern float turn_dir;
extern uint8_t obstacle_confidence;
extern uint8_t wall_detected;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Task_Control */
osThreadId_t Task_ControlHandle;
const osThreadAttr_t Task_Control_attributes = {
    .name = "Task_Control",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};
/* Definitions for Task_Sensor */
osThreadId_t Task_SensorHandle;
const osThreadAttr_t Task_Sensor_attributes = {
    .name = "Task_Sensor",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for Task_Decision */
osThreadId_t Task_DecisionHandle;
const osThreadAttr_t Task_Decision_attributes = {
    .name = "Task_Decision",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Task_Control */
  Task_ControlHandle = osThreadNew(StartTask02, NULL, &Task_Control_attributes);

  /* creation of Task_Sensor */
  Task_SensorHandle = osThreadNew(StartTask03, NULL, &Task_Sensor_attributes);

  /* creation of Task_Decision */
  Task_DecisionHandle =
      osThreadNew(StartTask04, NULL, &Task_Decision_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task_Control thread.
* @param argument: Not used
* @retval None
  任务 1：底层控制大脑 (10ms 绝对实时周期)替代了原来的 flag_10ms，负责按住 PID
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */

  osDelay(500);
  Motor_Init();   // 电机初始化
  HCSR04_Init();  // 超声波初始化
  Bumper_Init();  // 碰撞开关初始化
  Cliff_Init();   // 悬崖传感器初始化
  MPU6050_Init(); // MPU6050 初始化

  osDelay(500);        // 给 MPU6050 稳定时间
  MPU6050_Calibrate(); // MPU6050 校准（这里内部用了 HAL_Delay）
  Navi_Init();         // 导航初始化

  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 启动编码器
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  /* Infinite loop */
  for (;;)
  {
    osDelay(10);   // 精准 10ms
    Navi_Update(); // 姿态解算 + PID闭环
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Task_Sensor thread.
* @param argument: Not used
* @retval None
  任务 2：环境感知雷达 (50ms 周期) 替代了原来的 last_sonar_time 逻辑
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for (;;)
  {
    osDelay(50);

    float dist = HCSR04_Read();

    // 信心计数器逻辑
    if (dist > 2.0f && dist < 15.0f)
    {
      if (obstacle_confidence < 10)
        obstacle_confidence++;
    }
    else
    {
      if (obstacle_confidence > 0)
        obstacle_confidence--;
    }

    if (obstacle_confidence >= 3)
    {
      wall_detected = 1; // 发现墙壁，拉响全局警报
      obstacle_confidence = 0;
    }
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
 * @brief Function implementing the Task_Decision thread.
 * @param argument: Not used
 * @retval None
  任务 3：路径规划司令部 (100ms 周期) 弓字形算法、悬崖、物理碰撞的决策
 */
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for (;;)
  {
    // 决策周期 50ms，与超声波刷新率完美同步
    osDelay(50);

    // ==========================================
    // 优先级 1：悬崖保命（绝对生死底线）
    // ==========================================
    if (Cliff_GetState() != CLIFF_SAFE)
    {
      Navi_Escape_Backward(-35, 600); // 倒车，非阻塞退 600ms
      Navi_Turn_Angle(150.0f);        // 大角度甩尾逃生

      Cliff_ClearState(); // 清除警报
      zigzag_state = 0;   // 重新开始直行巡航
      continue; // 危险处理完毕，直接跳入下一轮循环（不往下执行）
    }

    // ==========================================
    // 优先级 2：物理碰撞微动开关（超声波盲区防线）
    // ==========================================
    if (Bumper_GetState() != BUMPER_NONE)
    {
      Navi_Escape_Backward(-30, 500);
      Navi_Turn_Angle(-150.0f);

      Bumper_ClearState();
      zigzag_state = 0;
      turn_dir = 1.0f; // 撞墙后重置下一次的默认转弯方向
      continue;
    }

    // ==========================================
    // 优先级 3 & 4：超声波避障与弓字形状态机
    // ==========================================
    float dist = HCSR04_Read(); // 获取安全的测距数据

    // 情况 A：直行巡航时遇到障碍物（触发 U 型弯的第一转）
    if (dist < 20.0f && dist > 0.0f && zigzag_state == 0)
    {
      Navi_Set_Straight(0); // 1. 刹车停稳
      osDelay(200);

      Navi_Escape_Backward(-30, 300); // 2. 稍微倒退，给车头留出转弯扫掠空间

      Navi_Turn_Angle(90.0f * turn_dir); // 3. 第一次 90 度原地转弯
      osDelay(200);

      // 4. 清空底层里程计，准备进入横向平移测量阶段
      global_mileage_L = 0;
      global_mileage_R = 0;

      zigzag_state = 2; // 切换到状态 2（平移换道）
    }
    // 情况 B：横向平移时竟然遇到障碍物（卡进死角或遇到窄巷）
    else if (dist < 20.0f && dist > 0.0f && zigzag_state == 2)
    {
      Navi_Set_Straight(0);
      osDelay(200);

      Navi_Escape_Backward(-30, 400);
      Navi_Turn_Angle(180.0f); // 放弃换道，直接掉头逃离
      osDelay(200);

      zigzag_state = 0; // 重新开始直行
    }
    // 情况 C：前方开阔安全，执行常规的弓字形清扫动作
    else
    {
      if (zigzag_state == 0)
      {
        // 状态 0：安心直线巡航清扫
        Navi_Set_Straight(30);
      }
      else if (zigzag_state == 2)
      {
        // 状态 2：横向平移换道
        Navi_Set_Straight(25);

        // 获取编码器平均脉冲数计算平移距离
        int32_t avg_pulses = (global_mileage_L + global_mileage_R) / 2;

        // 当脉冲数超过 1659 (约等于 16cm 车体宽度) 时，停止平移
        if (avg_pulses > 1659)
        {
          Navi_Set_Straight(0); // 刹车
          osDelay(200);
          zigzag_state = 3; // 触发换道结束的第二次转弯
        }
      }
      else if (zigzag_state == 3)
      {
        // 状态 3：换道末尾的第二次 90 度转弯
        Navi_Turn_Angle(90.0f * turn_dir);
        osDelay(200);

        // 转完之后，将下一次遇墙时的转弯方向取反
        turn_dir = -turn_dir;
        zigzag_state = 0; // 无缝切回直行清扫状态
      }
    }
  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ========================================================= */
/* 利用 FreeRTOS 滴答钩子喂养 HAL 库心跳 */
/* ========================================================= */
void vApplicationTickHook(void)
{
  // 每次 FreeRTOS 心跳 (1ms) 时，帮 HAL 库加一下时间
  //  HAL_Delay() 彻底解冻了，不需要额外的硬件定时器！
  HAL_IncTick();
}

/* USER CODE END Application */
