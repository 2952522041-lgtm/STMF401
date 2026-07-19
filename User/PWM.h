#pragma once

#include "main.h"
#include "tim.h"

typedef enum
{
    PWM_CH_FRONT_LEFT = 0,
    PWM_CH_FRONT_RIGHT,
    PWM_CH_BACK_LEFT,
    PWM_CH_BACK_RIGHT,
    PWM_CH_NUM,
} PWM_Channel_t;

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} PWM_Config_t;

void PWM_Init(void);
void Set_PWM_Duty(PWM_Channel_t channel, uint32_t duty);
uint32_t PWM_GetMaxDuty(PWM_Channel_t channel);
