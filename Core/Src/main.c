/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "motor.h"
#include "hc_sr04.h"
#include "bumper.h"
#include "cliff.h"
#include "mpu6050.h"
#include "navi.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint8_t is_crashed = 0; // 碰撞标志位（必须加 volatile 防优化）

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
		HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // 在检测传感器之前，强行等它 500 毫秒！
  // 给电容充电、给传感器内部芯片启动留足时间
  HAL_Delay(500);

  // 1. 初始化各个模块
  Motor_Init();  // 开启电机 PWM 定时器
  HCSR04_Init(); // 开启超声波定时器
  Bumper_Init(); // 初始化碰撞状态
  Cliff_Init();  // 初始化跌落状态

  MPU6050_Init();

  // 2. 机械防抖期  2.5 秒的时间把手彻底拿开
  HAL_Delay(2500);

  if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 3, 100) != HAL_OK)
  {
    while (1)
    {
      Motor_SetSpeed(30, -30);
      HAL_Delay(100);
      Motor_SetSpeed(0, 0);
      HAL_Delay(500);
    }
  }

  // 3. 算法校准期
  MPU6050_Calibrate(); // 耗时约 1 秒钟计算零漂

  // 4. 起跑线对齐
  Navi_Init();
  //  启动编码器定时器
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 启动左轮编码器
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // 启动右轮编码器

  // 在进入死循环前，记录真正的起跑时间！
  uint32_t state_start_time = HAL_GetTick();

  // 弓字形状态机专属变量
  uint8_t zigzag_state = 0; // 0:巡航直行, 1:换道第一转, 2:换道平移, 3:换道第二转
  float turn_dir = 1.0f;    // 转向乘数：1.0f代表向左U-Turn，-1.0f代表向右U-Turn
  uint32_t last_sonar_time = 0; // 超声波读取节拍器
  uint8_t obstacle_confidence = 0;// 避障信心计数器

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // =====================================
    // 0. 底层心脏：10ms 姿态与速度闭环
    // =====================================
    if (flag_10ms == 1)
    {
      flag_10ms = 0;
      Navi_Update();
    }

    // =====================================
    // 1. 绝对最高优先级：悬崖保命 (防跌落)
    // =====================================
    if (Cliff_GetState() != CLIFF_SAFE)
    {
      // 不丢失 PID 心跳地后退 600ms
      Navi_Escape_Backward(-35, 600);

      Navi_Turn_Angle(150.0f);
      Cliff_ClearState();

      // 遭遇生死危机，打断弓字形，重新开始规划
      zigzag_state = 0;
    }

    // =====================================
    // 2. 次高优先级：物理碰撞 (微动开关)
    // =====================================
    else if (Bumper_GetState() != BUMPER_NONE)
    {
      Navi_Escape_Backward(-30, 500); // 倒车脱困
      Navi_Turn_Angle(-150.0f);       // 假设固定向右转逃离
      Bumper_ClearState();

      // 被撞击说明前方路线被破坏，重置弓字形
      zigzag_state = 0;
      turn_dir = 1.0f;
      obstacle_confidence = 0;
    }

    // =====================================
    // 3. 优先级分支：读取超声波 (环境感知层)
    // =====================================
    else
    {
      uint8_t wall_detected = 0;              // 墙壁检测标志位
    
      // 每 60ms 探测一次前方距离，防止阻塞 PID
      if (HAL_GetTick() - last_sonar_time > 60)
      {
        last_sonar_time = HAL_GetTick();
        float dist = HCSR04_Read();

        // 发现疑似墙壁或障碍物 (假设 15cm)
        if (dist > 2.0f && dist < 15.0f)
        {
          if (obstacle_confidence < 10)
            obstacle_confidence++; // 信心累加
        }
        else
        {
          if (obstacle_confidence > 0)
            obstacle_confidence--; // 没看到就递减（容错）
        }

        // 只有信心爆棚（连续确认了 3 次），才真正宣告发现墙壁！
        // 这就能完美过滤掉抹布、偶尔的声波反射丢失等假信号
        if (obstacle_confidence >= 3)
        {
          wall_detected = 1;
          obstacle_confidence = 0; // 触发后立刻清零，防止连续触发
        }
      }

      // =====================================
      // 4. 最低优先级：弓字形状态机 (路径规划层)
      // =====================================

      // 如果探测到墙壁
      if (wall_detected)
      {
        if (zigzag_state == 0)
        {
          // 状态0遇到墙，说明一条直线扫完了，触发换道！
          Navi_Set_Straight(0); // 刹车
          zigzag_state = 1;     // 进入转向状态
        }
        else if (zigzag_state == 2)
        {
          // 极品死角处理：如果在平移换道时也遇到了墙，说明卡在角落了！
          Navi_Escape_Backward(-30, 500);
          Navi_Turn_Angle(180.0f); // 直接原地掉头
          zigzag_state = 0;        // 重新开始扫
        }
      }

      // 如果前方安全，严格执行弓字形动作
      else
      {
        if (zigzag_state == 0)
        {
          // 状态0：无脑巡航直线清扫
          Navi_Set_Straight(30); // 设定扫地巡航速度
        }
        else if (zigzag_state == 1)
        {
          // 状态1：第一次 90 度转向
          Navi_Turn_Angle(90.0f * turn_dir);

          // 【核心动作】转向完成准备平移前，把里程计清零！
          global_mileage_L = 0;
          global_mileage_R = 0;

          zigzag_state = 2;
        }
        else if (zigzag_state == 2)
        {
          // 状态2：侧向平移 (走过车身宽度的距离，相当于换车道)
          Navi_Set_Straight(25);

          // 计算双轮平均走过的脉冲数
          int32_t avg_pulses = (global_mileage_L + global_mileage_R) / 2;

          // 当平均脉冲数超过 1659 (即完美走过 16cm) 时，停止平移！
          if (avg_pulses > 1659)
          {
            zigzag_state = 3;
          }
        }
        else if (zigzag_state == 3)
        {
          // 状态3：第二次 90 度转向 (完成 U 型弯)
          Navi_Turn_Angle(90.0f * turn_dir);

          // 极其关键：下一次换道必须反方向转！(向左变成向右)
          turn_dir = -turn_dir;

          // 换道彻底完成，进入直行扫地
          zigzag_state = 0;
        }
      }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
