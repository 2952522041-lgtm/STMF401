#include "encode.h"
#include "tim.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t last_count;
    float rpm;
    float direction;
} Encoder_Config_t;

static Encoder_Config_t encoders[ENCODER_NUM] =
{
    [ENCODER_FRONT_LEFT] = {&htim1, 0u, 0.0f, ENCODER_FRONT_LEFT_DIRECTION},
    [ENCODER_FRONT_RIGHT] = {&htim4, 0u, 0.0f, ENCODER_FRONT_RIGHT_DIRECTION},
    [ENCODER_BACK_LEFT] = {&htim2, 0u, 0.0f, ENCODER_BACK_LEFT_DIRECTION},
    [ENCODER_BACK_RIGHT] = {&htim3, 0u, 0.0f, ENCODER_BACK_RIGHT_DIRECTION},
};

static int32_t Encoder_GetDelta(const Encoder_Config_t *encoder, uint32_t now)
{
    if (__HAL_TIM_GET_AUTORELOAD(encoder->htim) > 0xFFFFu)
    {
        return (int32_t)(now - encoder->last_count);
    }

    return (int32_t)(int16_t)((uint16_t)now - (uint16_t)encoder->last_count);
}

void Encoder_Init(void)
{
    for (uint32_t i = 0; i < ENCODER_NUM; i++)
    {
        if (HAL_TIM_Encoder_Start(encoders[i].htim, TIM_CHANNEL_ALL) != HAL_OK)
        {
            Error_Handler();
        }
    }

    Encoder_ResetAll();
}

void Encoder_Reset(Encoder_ID_t encoder)
{
    if (encoder >= ENCODER_NUM)
    {
        return;
    }

    __HAL_TIM_SET_COUNTER(encoders[encoder].htim, 0u);
    encoders[encoder].last_count = 0u;
    encoders[encoder].rpm = 0.0f;
}

void Encoder_ResetAll(void)
{
    for (uint32_t i = 0; i < ENCODER_NUM; i++)
    {
        Encoder_Reset((Encoder_ID_t)i);
    }
}

int32_t Encoder_GetCount(Encoder_ID_t encoder)
{
    if (encoder >= ENCODER_NUM)
    {
        return 0;
    }

    uint32_t count = __HAL_TIM_GET_COUNTER(encoders[encoder].htim);

    if (__HAL_TIM_GET_AUTORELOAD(encoders[encoder].htim) > 0xFFFFu)
    {
        return (int32_t)count;
    }

    return (int32_t)(int16_t)(uint16_t)count;
}

void Encoder_Update(float dt_sec)
{
    if (dt_sec <= 0.0f)
    {
        return;
    }

    for (uint32_t i = 0; i < ENCODER_NUM; i++)
    {
        uint32_t now = __HAL_TIM_GET_COUNTER(encoders[i].htim);
        int32_t diff = Encoder_GetDelta(&encoders[i], now);
        encoders[i].last_count = now;

        float wheel_revolutions = (float)diff / (ENCODER_PPR * ENCODER_QUAD_MULTIPLY * MOTOR_GEAR_RATIO);
        encoders[i].rpm = wheel_revolutions * 60.0f / dt_sec * encoders[i].direction;
    }
}

float Encoder_GetRPM(Encoder_ID_t encoder)
{
    if (encoder >= ENCODER_NUM)
    {
        return 0.0f;
    }

    return encoders[encoder].rpm;
}
