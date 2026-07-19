#pragma once

#include "main.h"

#define UARTMODEL_MOTOR_NUM 4u
#define UARTMODEL_FRAME_HEADER_1 0xAAu
#define UARTMODEL_FRAME_HEADER_2 0x55u
#define UARTMODEL_CMD_SET_TARGET_RPM 0x01u
#define UARTMODEL_CMD_STOP 0x02u
#define UARTMODEL_MSG_TELEMETRY 0x81u

/*
 * Frame: AA 55 TYPE LENGTH PAYLOAD CHECKSUM
 * CHECKSUM is the XOR of TYPE, LENGTH and every payload byte.
 * SET_TARGET_RPM payload: FL, FR, BL, BR as little-endian int16, unit 0.1 RPM.
 * TELEMETRY payload: four target int16, four measured int16 and four count int32.
 */
void UARTMODEL_Init(void);
uint8_t UARTMODEL_GetTargetRPM(float target_rpm[UARTMODEL_MOTOR_NUM]);
HAL_StatusTypeDef UARTMODEL_SendTelemetry(const float target_rpm[UARTMODEL_MOTOR_NUM],
                                          const float measured_rpm[UARTMODEL_MOTOR_NUM],
                                          const int32_t encoder_count[UARTMODEL_MOTOR_NUM]);
