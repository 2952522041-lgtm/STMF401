#pragma once

typedef enum
{
    MOTOR_FRONT_LEFT = 0,
    MOTOR_FRONT_RIGHT,
    MOTOR_BACK_LEFT,
    MOTOR_BACK_RIGHT,
    MOTOR_NUM,
} Motor_ID_t;

void Motor_Init(void);

void Motor_SetRPM(Motor_ID_t motor, float rpm);
void Motor_SetAllRPM(float front_left_rpm, float front_right_rpm, float back_left_rpm, float back_right_rpm);

void Motor_SetTargetRPM(Motor_ID_t motor, float rpm);
void Motor_SetAllTargetRPM(float front_left_rpm, float front_right_rpm, float back_left_rpm, float back_right_rpm);
float Motor_GetTargetRPM(Motor_ID_t motor);
void Motor_GetAllTargetRPM(float target_rpm[MOTOR_NUM]);

void Motor_SetSpeedPercent(Motor_ID_t motor, float percent);
void Motor_SetAllSpeedPercent(float front_left_percent, float front_right_percent, float back_left_percent, float back_right_percent);

void Motor_Stop(Motor_ID_t motor);
void Motor_StopAll(void);
void Motor_Brake(Motor_ID_t motor);
void Motor_BrakeAll(void);
