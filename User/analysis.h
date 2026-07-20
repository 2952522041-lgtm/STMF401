#pragma once

#include "main.h"

#define ANALYSIS_PACKET_LENGTH 5u
#define ANALYSIS_TIMEOUT_MS 300u

#define ANALYSIS_SPEED_1_RPM 100.0f
#define ANALYSIS_SPEED_2_RPM 200.0f
#define ANALYSIS_SPEED_3_RPM 300.0f

typedef struct
{
    float front_left_rpm;
    float front_right_rpm;
    float back_left_rpm;
    float back_right_rpm;
    uint32_t tick_ms;
    uint8_t valid;
} Analysis_TargetRPM_t;

void Analysis_Init(void);
void Analysis_StartUartReceive(void);
void Analysis_UartRxCpltCallback(UART_HandleTypeDef *huart);
void Analysis_UartErrorCallback(UART_HandleTypeDef *huart);
void Analysis_InputByte(uint8_t byte);
void Analysis_Update(void);
uint8_t Analysis_GetTargetRPM(Analysis_TargetRPM_t *target);
uint8_t Analysis_IsTargetTimeout(uint32_t now_ms);
