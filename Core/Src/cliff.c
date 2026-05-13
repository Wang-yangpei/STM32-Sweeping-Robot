#include "cliff.h"

static volatile Cliff_ID_t current_cliff_state = CLIFF_SAFE;

void Cliff_Init(void)
{
    current_cliff_state = CLIFF_SAFE;
}

Cliff_ID_t Cliff_GetState(void)
{
    return current_cliff_state;
}

void Cliff_ClearState(void)
{
    current_cliff_state = CLIFF_SAFE;
}

__weak void Cliff_EventCallback(Cliff_ID_t id)
{
    (void)id;
}

void Cliff_EXTI_Handler(uint16_t GPIO_Pin)
{
    Cliff_ID_t trigger_id = CLIFF_SAFE;

    if (GPIO_Pin == CLIFF_LEFT_Pin) {
        trigger_id = CLIFF_LEFT;
    } else if (GPIO_Pin == CLIFF_FRONT_Pin) {
        trigger_id = CLIFF_FRONT;
    } else if (GPIO_Pin == CLIFF_RIGHT_Pin) {
        trigger_id = CLIFF_RIGHT;
    }

    if (trigger_id != CLIFF_SAFE) 
    {
        current_cliff_state |= trigger_id;
        Cliff_EventCallback(trigger_id);
    }
}
