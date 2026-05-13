#include "hc_sr04.h"
#include "motor.h"

void HCSR04_Init(void)
{
    HAL_TIM_Base_Start(&htim3);
}


// 1. 微秒级延时函数 (利用 TIM3)
void delay_us(uint16_t us) 
{
    __HAL_TIM_SET_COUNTER(&htim3, 0); // 秒表清零
    while (__HAL_TIM_GET_COUNTER(&htim3) < us); // 死等
}

// 2. 超声波测距核心函数
float HCSR04_Read(void) 
{
    uint32_t local_time = 0;
    float distance = 0;
    uint32_t timeout = 0;

    // 第一步：发声 (Trig -> PB11)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); 
    delay_us(15);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

    //Echo 引脚在发送瞬间会变高，在收到回声时变低
    // 第二步：等回声开头 (Echo -> PB10)
    while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) {
        timeout++;
        if(timeout > 100000) return 999.0; // 超时保护
    }

    // 第三步：按下秒表
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    // 第四步：等回声结束
    timeout = 0;
    while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) {
        timeout++;
        if(timeout > 100000) return 999.0; // 超时保护
    }

    // 第五步：看秒表
    local_time = __HAL_TIM_GET_COUNTER(&htim3);

    // 第六步：算距离 (厘米)
    distance = local_time * 0.017f; 

    return distance; 
}

// 3. 智能避障核心任务
void HCSR04_AvoidanceTask(void)
{
    float dist = HCSR04_Read(); 
      
    // 设定安全距离为 20 厘米
    if (dist < 20.0f && dist > 0.0f) 
    {
        // 发现障碍物，触发脱困连招！
        Robot_Stop();            
        HAL_Delay(200);          
        
        Robot_MoveBackward(400); 
        HAL_Delay(300);
        
        Robot_SpinLeft(500);     
        HAL_Delay(600);          
        
        Robot_Stop();            
        HAL_Delay(200);
    }
    else 
    {
        // 前方开阔，愉快扫地
        Robot_MoveForward(450);  
        HAL_Delay(50);           
    }
}
