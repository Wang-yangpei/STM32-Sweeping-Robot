#ifndef __HC_SR04_H
#define __HC_SR04_H

#include "main.h"
#include "tim.h"  // 要用到 htim3 这个秒表，所以必须包含它

void HCSR04_Init(void);

// 声明微秒级延时函数
void delay_us(uint16_t us);

// 声明超声波读取函数
float HCSR04_Read(void);

// 避障任务函数
void HCSR04_AvoidanceTask(void);

#endif
