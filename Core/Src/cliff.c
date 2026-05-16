#include "cliff.h"

static volatile Cliff_ID_t current_cliff_state = CLIFF_SAFE;

void Cliff_Init(void) { current_cliff_state = CLIFF_SAFE; }

Cliff_ID_t Cliff_GetState(void) { return current_cliff_state; }

void Cliff_ClearState(void) { current_cliff_state = CLIFF_SAFE; }

__weak void Cliff_EventCallback(Cliff_ID_t id) { (void)id; }

void Cliff_EXTI_Handler(uint16_t GPIO_Pin)
{
  // 静态变量记录上一次触发的时间
  static uint32_t last_trigger_time = 0;
  uint32_t current_time = HAL_GetTick();

  // 滤噪逻辑：如果两次中断触发的间隔小于 20ms
  // 直接判定为临界点闪烁，忽略不计 保护系统不被边缘的高频中断淹没
  if ((current_time - last_trigger_time) < 20)
  {
    return;
  }
  last_trigger_time = current_time;

  Cliff_ID_t trigger_id = CLIFF_SAFE;

  if (GPIO_Pin == CLIFF_LEFT_Pin)
  {
    trigger_id = CLIFF_LEFT;
  }
  else if (GPIO_Pin == CLIFF_FRONT_Pin)
  {
    trigger_id = CLIFF_FRONT;
  }
  else if (GPIO_Pin == CLIFF_RIGHT_Pin)
  {
    trigger_id = CLIFF_RIGHT;
  }

  if (trigger_id != CLIFF_SAFE)
  {
    current_cliff_state |= trigger_id;
    Cliff_EventCallback(trigger_id); // 触发回调，执行 Robot_Stop() 瞬间保命
  }
}
