#include "hc_sr04.h"
#include "motor.h"

void HCSR04_Init(void) { HAL_TIM_Base_Start(&htim3); }

// 1. 微秒级延时函数 (利用 TIM3)
void delay_us(uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim3, 0); // 秒表清零
  while (__HAL_TIM_GET_COUNTER(&htim3) < us)
    ; // 死等
}

// 2. 超声波测距核心函数
float HCSR04_Read(void)
{
  uint32_t tickstart;
  float distance;

  // 1. 发送 15us 的高电平触发信号
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
  delay_us(15);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

  // 2. 等待 Echo 变高电平（加入 30ms 超时防死锁）
  tickstart = HAL_GetTick();
  while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET)
  {
    if ((HAL_GetTick() - tickstart) > 30)
    {
      return 999.0f; // 硬件异常或超时，直接弹回安全值，保护 Task_Sensor 不卡死
    }
  }

  // 3. 抓到高电平瞬间，开启秒表（硬件定时器清零）
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  // 4. 等待 Echo 变低电平（加入 30ms 超时防死锁）
  tickstart = HAL_GetTick();
  while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET)
  {
    if ((HAL_GetTick() - tickstart) > 30)
    {
      return 999.0f; // 超出最大射程或异常
    }
  }

  // 5. 抓到低电平瞬间，立刻读取微秒数
  uint32_t raw_time = __HAL_TIM_GET_COUNTER(&htim3);

  // 6. 换算距离
  distance = (float)raw_time * 0.017f;

  return distance;
}


