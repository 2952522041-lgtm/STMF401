#include "PWM.h"

static const PWM_Config_t PWM_CHANNELS[PWM_CH_NUM] =
{
    [PWM_CH_FRONT_LEFT] = {&htim5, TIM_CHANNEL_1},
    [PWM_CH_FRONT_RIGHT] = {&htim5, TIM_CHANNEL_2},
    [PWM_CH_BACK_LEFT] = {&htim5, TIM_CHANNEL_3},
    [PWM_CH_BACK_RIGHT] = {&htim5, TIM_CHANNEL_4},
};

void PWM_Init(void)
{
    for (uint32_t i = 0; i < PWM_CH_NUM; i++)
    {
        if (HAL_TIM_PWM_Start(PWM_CHANNELS[i].htim, PWM_CHANNELS[i].channel) != HAL_OK)
        {
            Error_Handler();
        }

        Set_PWM_Duty((PWM_Channel_t)i, 0u);
    }
}

void Set_PWM_Duty(PWM_Channel_t channel, uint32_t duty)
{
    if (channel >= PWM_CH_NUM)
    {
        return;
    }

    TIM_HandleTypeDef *htim = PWM_CHANNELS[channel].htim;
    uint32_t tim_channel = PWM_CHANNELS[channel].channel;
    uint32_t max_duty = PWM_GetMaxDuty(channel);

    if (duty > max_duty)
    {
        duty = max_duty;
    }

    __HAL_TIM_SET_COMPARE(htim, tim_channel, duty);
}

uint32_t PWM_GetMaxDuty(PWM_Channel_t channel)
{
    if (channel >= PWM_CH_NUM)
    {
        return 0u;
    }

    return __HAL_TIM_GET_AUTORELOAD(PWM_CHANNELS[channel].htim);
}
