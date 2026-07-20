#include "tb6612.h"
#include "PWM.h"

typedef struct
{
    GPIO_TypeDef *IN1_GPIO_Port;
    uint16_t IN1_Pin;
    GPIO_TypeDef *IN2_GPIO_Port;
    uint16_t IN2_Pin;
    PWM_Channel_t pwm_channel;
    float direction;
} tb6612_Config_t;

static const tb6612_Config_t tb6612s[tb6612_CH_NUM] =
{
    [tb6612_CH_FRONT_LEFT] = {AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin, PWM_CH_FRONT_LEFT, TB6612_FRONT_LEFT_DIRECTION},
    [tb6612_CH_FRONT_RIGHT] = {BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin, PWM_CH_FRONT_RIGHT, TB6612_FRONT_RIGHT_DIRECTION},
    [tb6612_CH_BACK_LEFT] = {CIN1_GPIO_Port, CIN1_Pin, CIN2_GPIO_Port, CIN2_Pin, PWM_CH_BACK_LEFT, TB6612_BACK_LEFT_DIRECTION},
    [tb6612_CH_BACK_RIGHT] = {DIN1_GPIO_Port, DIN1_Pin, DIN2_GPIO_Port, DIN2_Pin, PWM_CH_BACK_RIGHT, TB6612_BACK_RIGHT_DIRECTION},
};

static tb6612_Direction_t current_direction[tb6612_CH_NUM] =
{
    tb6612_DIR_STOP,
    tb6612_DIR_STOP,
    tb6612_DIR_STOP,
    tb6612_DIR_STOP,
};

static float tb6612_abs(float number)
{
    return (number < 0.0f) ? -number : number;
}

void tb6612_Init(void)
{
    PWM_Init();

    for (uint32_t i = 0; i < tb6612_CH_NUM; i++)
    {
        tb6612_Stop((tb6612_Channel_t)i);
    }
}

void tb6612_SetDuty(tb6612_Channel_t channel, uint32_t duty)
{
    if (channel >= tb6612_CH_NUM)
    {
        return;
    }

    Set_PWM_Duty(tb6612s[channel].pwm_channel, duty);
}

void tb6612_SetDirection(tb6612_Channel_t channel, tb6612_Direction_t direction)
{
    if (channel >= tb6612_CH_NUM)
    {
        return;
    }

    GPIO_TypeDef *IN1_GPIO_Port = tb6612s[channel].IN1_GPIO_Port;
    uint16_t IN1_Pin = tb6612s[channel].IN1_Pin;
    GPIO_TypeDef *IN2_GPIO_Port = tb6612s[channel].IN2_GPIO_Port;
    uint16_t IN2_Pin = tb6612s[channel].IN2_Pin;

    switch (direction)
    {
        case tb6612_DIR_FORWARD:
            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
            break;

        case tb6612_DIR_BACKWARD:
            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
            break;

        case tb6612_DIR_BRAKE:
            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
            break;

        case tb6612_DIR_STOP:
        default:
            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
            direction = tb6612_DIR_STOP;
            break;
    }

    current_direction[channel] = direction;
}

void tb6612_Stop(tb6612_Channel_t channel)
{
    tb6612_SetDuty(channel, 0u);
    tb6612_SetDirection(channel, tb6612_DIR_STOP);
}

void tb6612_Brake(tb6612_Channel_t channel)
{
    tb6612_SetDuty(channel, 0u);
    tb6612_SetDirection(channel, tb6612_DIR_BRAKE);
}

void tb6612_SetRPM(tb6612_Channel_t channel, float rpm)
{
    if (channel >= tb6612_CH_NUM)
    {
        return;
    }

    float motor_rpm;
    tb6612_Direction_t new_direction;

    if (rpm != rpm)
    {
        tb6612_Stop(channel);
        return;
    }

    motor_rpm = rpm * tb6612s[channel].direction;

    if (motor_rpm == 0.0f)
    {
        tb6612_Stop(channel);
        return;
    }

    if (motor_rpm > tb6612_max_rpm)
    {
        motor_rpm = tb6612_max_rpm;
    }
    else if (motor_rpm < -tb6612_max_rpm)
    {
        motor_rpm = -tb6612_max_rpm;
    }

    uint32_t max_duty = PWM_GetMaxDuty(tb6612s[channel].pwm_channel);
    uint32_t duty = (uint32_t)((tb6612_abs(motor_rpm) / tb6612_max_rpm) * (float)max_duty);

    if (motor_rpm > 0.0f)
    {
        new_direction = tb6612_DIR_FORWARD;
    }
    else
    {
        new_direction = tb6612_DIR_BACKWARD;
    }

    if (new_direction != current_direction[channel])
    {
        /* Remove PWM before changing H-bridge polarity. */
        tb6612_SetDuty(channel, 0u);
        tb6612_SetDirection(channel, new_direction);
    }

    tb6612_SetDuty(channel, duty);
}
