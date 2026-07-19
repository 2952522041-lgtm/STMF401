#pragma once

#include "main.h"

#define tb6612_max_rpm 330.0f

/* Change one of these signs if a wheel rotates in the opposite direction. */
#define TB6612_FRONT_LEFT_DIRECTION 1.0f
#define TB6612_FRONT_RIGHT_DIRECTION 1.0f
#define TB6612_BACK_LEFT_DIRECTION 1.0f
#define TB6612_BACK_RIGHT_DIRECTION 1.0f

typedef enum
{
    tb6612_DIR_STOP = 0,
    tb6612_DIR_FORWARD,
    tb6612_DIR_BACKWARD,
    tb6612_DIR_BRAKE,
} tb6612_Direction_t;

typedef enum
{
    tb6612_CH_FRONT_LEFT = 0,
    tb6612_CH_FRONT_RIGHT,
    tb6612_CH_BACK_LEFT,
    tb6612_CH_BACK_RIGHT,
    tb6612_CH_NUM,
} tb6612_Channel_t;

void tb6612_Init(void);
void tb6612_SetDuty(tb6612_Channel_t channel, uint32_t duty);
void tb6612_SetDirection(tb6612_Channel_t channel, tb6612_Direction_t direction);
void tb6612_Stop(tb6612_Channel_t channel);
void tb6612_Brake(tb6612_Channel_t channel);
void tb6612_SetRPM(tb6612_Channel_t channel, float rpm);
