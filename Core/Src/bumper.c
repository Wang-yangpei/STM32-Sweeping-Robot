#include "bumper.h"

// 静态全局变量，对外隐藏内部状态，防止被外部随意篡改
static volatile Bumper_ID_t current_bumper_state = BUMPER_NONE;

void Bumper_Init(void)
{
    current_bumper_state = BUMPER_NONE;
    // 具体的 GPIO 初始化由 CubeMX 的 MX_GPIO_Init 完成，这里主要做软件状态复位
}

// 获取当前碰撞状态
Bumper_ID_t Bumper_GetState(void)
{
    return current_bumper_state;
}

// 清除碰撞状态（脱困完成后由主程序调用）
void Bumper_ClearState(void)
{
    current_bumper_state = BUMPER_NONE;
}

// 弱函数：默认不执行任何操作。用户需要在 main.c 中重写此函数
__weak void Bumper_EventCallback(Bumper_ID_t id)
{
    // 防止编译器报“未使用变量”的警告
    (void)id; 
}

// 核心中断处理逻辑（放在 HAL_GPIO_EXTI_Callback 中调用）
void Bumper_EXTI_Handler(uint16_t GPIO_Pin)
{
    Bumper_ID_t trigger_id = BUMPER_NONE;

    // 判断是哪一个引脚触发了中断
    if (GPIO_Pin == BUMPER_LEFT_Pin) {
        trigger_id = BUMPER_LEFT;
    } else if (GPIO_Pin == BUMPER_FRONT_Pin) {
        trigger_id = BUMPER_FRONT;
    } else if (GPIO_Pin == BUMPER_RIGHT_Pin) {
        trigger_id = BUMPER_RIGHT;
    }

    // 如果确实是微动开关触发的
    if (trigger_id != BUMPER_NONE) 
    {
        current_bumper_state |= trigger_id; // 记录状态（支持多个同时按下）
        Bumper_EventCallback(trigger_id);   // 抛出事件给应用层
    }
}
