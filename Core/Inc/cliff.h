#ifndef __CLIFF_H
#define __CLIFF_H

#include "main.h"
#include <stdint.h>

typedef enum {
    CLIFF_SAFE  = 0x00,
    CLIFF_LEFT  = 0x01,
    CLIFF_FRONT = 0x02,
    CLIFF_RIGHT = 0x04
} Cliff_ID_t;

void Cliff_Init(void);
Cliff_ID_t Cliff_GetState(void);
void Cliff_ClearState(void);
void Cliff_EXTI_Handler(uint16_t GPIO_Pin);
void Cliff_EventCallback(Cliff_ID_t id);

#endif /* __CLIFF_H */
