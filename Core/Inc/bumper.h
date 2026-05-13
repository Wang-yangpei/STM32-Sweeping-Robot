#ifndef __BUMPER_H
#define __BUMPER_H

#include "main.h"
#include <stdint.h>

// 使用枚举定义碰撞来源，支持按位或（如果同时撞到两个开关）
typedef enum {
    BUMPER_NONE  = 0x00,
    BUMPER_LEFT  = 0x01,
    BUMPER_FRONT = 0x02,
    BUMPER_RIGHT = 0x04
} Bumper_ID_t;

// API 接口声明
void Bumper_Init(void);
Bumper_ID_t Bumper_GetState(void);
void Bumper_ClearState(void);

// 供外部中断调用的处理函数
void Bumper_EXTI_Handler(uint16_t GPIO_Pin);

// 弱函数回调声明（供应用层重写）
void Bumper_EventCallback(Bumper_ID_t id);

#endif /* __BUMPER_H */
