#pragma once

#include "main.h"

#define ENCODER_PPR 13.0f
#define ENCODER_QUAD_MULTIPLY 4.0f
#define MOTOR_GEAR_RATIO 30.0f

/* Change one of these signs if its measured RPM polarity is reversed. */
#define ENCODER_FRONT_LEFT_DIRECTION 1.0f
#define ENCODER_FRONT_RIGHT_DIRECTION 1.0f
#define ENCODER_BACK_LEFT_DIRECTION 1.0f
#define ENCODER_BACK_RIGHT_DIRECTION 1.0f

typedef enum
{
    ENCODER_FRONT_LEFT = 0,
    ENCODER_FRONT_RIGHT,
    ENCODER_BACK_LEFT,
    ENCODER_BACK_RIGHT,
    ENCODER_NUM,
} Encoder_ID_t;

void Encoder_Init(void);
void Encoder_Reset(Encoder_ID_t encoder);
void Encoder_ResetAll(void);
int32_t Encoder_GetCount(Encoder_ID_t encoder);
void Encoder_Update(float dt_sec);
float Encoder_GetRPM(Encoder_ID_t encoder);
