#include "Motor.h"
#include "tb6612.h"

static float target_rpms[MOTOR_NUM] = {0.0f};

static const tb6612_Channel_t motor_map[MOTOR_NUM] =
{
    [MOTOR_FRONT_LEFT] = tb6612_CH_FRONT_LEFT,
    [MOTOR_FRONT_RIGHT] = tb6612_CH_FRONT_RIGHT,
    [MOTOR_BACK_LEFT] = tb6612_CH_BACK_LEFT,
    [MOTOR_BACK_RIGHT] = tb6612_CH_BACK_RIGHT,
};

static float Motor_Limit(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

void Motor_Init(void)
{
    tb6612_Init();
    Motor_StopAll();
}

void Motor_SetRPM(Motor_ID_t motor, float rpm)
{
    if (motor >= MOTOR_NUM)
    {
        return;
    }

    tb6612_SetRPM(motor_map[motor], rpm);
}

void Motor_SetAllRPM(float front_left_rpm, float front_right_rpm, float back_left_rpm, float back_right_rpm)
{
    Motor_SetRPM(MOTOR_FRONT_LEFT, front_left_rpm);
    Motor_SetRPM(MOTOR_FRONT_RIGHT, front_right_rpm);
    Motor_SetRPM(MOTOR_BACK_LEFT, back_left_rpm);
    Motor_SetRPM(MOTOR_BACK_RIGHT, back_right_rpm);
}

void Motor_SetTargetRPM(Motor_ID_t motor, float rpm)
{
    if (motor >= MOTOR_NUM)
    {
        return;
    }

    target_rpms[motor] = Motor_Limit(rpm, tb6612_max_rpm);
}

void Motor_SetAllTargetRPM(float front_left_rpm, float front_right_rpm, float back_left_rpm, float back_right_rpm)
{
    Motor_SetTargetRPM(MOTOR_FRONT_LEFT, front_left_rpm);
    Motor_SetTargetRPM(MOTOR_FRONT_RIGHT, front_right_rpm);
    Motor_SetTargetRPM(MOTOR_BACK_LEFT, back_left_rpm);
    Motor_SetTargetRPM(MOTOR_BACK_RIGHT, back_right_rpm);
}

float Motor_GetTargetRPM(Motor_ID_t motor)
{
    if (motor >= MOTOR_NUM)
    {
        return 0.0f;
    }

    return target_rpms[motor];
}

void Motor_SetSpeedPercent(Motor_ID_t motor, float percent)
{
    if (motor >= MOTOR_NUM)
    {
        return;
    }

    percent = Motor_Limit(percent, 100.0f);
    tb6612_SetRPM(motor_map[motor], percent * tb6612_max_rpm / 100.0f);
}

void Motor_SetAllSpeedPercent(float front_left_percent, float front_right_percent, float back_left_percent, float back_right_percent)
{
    Motor_SetSpeedPercent(MOTOR_FRONT_LEFT, front_left_percent);
    Motor_SetSpeedPercent(MOTOR_FRONT_RIGHT, front_right_percent);
    Motor_SetSpeedPercent(MOTOR_BACK_LEFT, back_left_percent);
    Motor_SetSpeedPercent(MOTOR_BACK_RIGHT, back_right_percent);
}

void Motor_Stop(Motor_ID_t motor)
{
    if (motor < MOTOR_NUM)
    {
        tb6612_Stop(motor_map[motor]);
    }
}

void Motor_StopAll(void)
{
    for (uint32_t i = 0; i < MOTOR_NUM; i++)
    {
        Motor_Stop((Motor_ID_t)i);
    }
}

void Motor_Brake(Motor_ID_t motor)
{
    if (motor < MOTOR_NUM)
    {
        tb6612_Brake(motor_map[motor]);
    }
}

void Motor_BrakeAll(void)
{
    for (uint32_t i = 0; i < MOTOR_NUM; i++)
    {
        Motor_Brake((Motor_ID_t)i);
    }
}
