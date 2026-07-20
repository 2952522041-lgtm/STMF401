#include "analysis.h"
#include "usart.h"

#include <string.h>

#define PACKET_HEADER ((uint8_t)'@')
#define PACKET_TAIL ((uint8_t)'#')
#define CRC8_POLY 0x07u

static uint8_t uart_rx_byte;
static uint8_t rx_build_packet[ANALYSIS_PACKET_LENGTH];
static volatile uint8_t rx_ready_packet[ANALYSIS_PACKET_LENGTH];
static uint8_t rx_index;
static volatile uint8_t packet_ready;
static volatile uint8_t uart_restart_required;
static Analysis_TargetRPM_t current_target;

static HAL_StatusTypeDef Analysis_StartByteReceive(void);
static void Analysis_RestartUartIfRequired(void);
static uint8_t Analysis_CRC8(const uint8_t *data, uint8_t length);
static uint8_t Analysis_CheckPacket(const uint8_t packet[ANALYSIS_PACKET_LENGTH]);
static void Analysis_CalculateTarget(uint8_t mode, uint8_t speed, Analysis_TargetRPM_t *target);

void Analysis_Init(void)
{
    memset(rx_build_packet, 0, sizeof(rx_build_packet));

    for (uint32_t i = 0; i < ANALYSIS_PACKET_LENGTH; i++)
    {
        rx_ready_packet[i] = 0u;
    }

    memset(&current_target, 0, sizeof(current_target));

    uart_rx_byte = 0u;
    rx_index = 0u;
    packet_ready = 0u;
    uart_restart_required = 0u;
}

void Analysis_StartUartReceive(void)
{
    if (HAL_UART_RegisterCallback(&huart6,
                                  HAL_UART_RX_COMPLETE_CB_ID,
                                  Analysis_UartRxCpltCallback) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_UART_RegisterCallback(&huart6,
                                  HAL_UART_ERROR_CB_ID,
                                  Analysis_UartErrorCallback) != HAL_OK)
    {
        Error_Handler();
    }

    if (Analysis_StartByteReceive() != HAL_OK)
    {
        Error_Handler();
    }
}

void Analysis_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART6))
    {
        Analysis_InputByte(uart_rx_byte);

        if (Analysis_StartByteReceive() != HAL_OK)
        {
            uart_restart_required = 1u;
        }
    }
}

void Analysis_UartErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART6))
    {
        uint32_t error = HAL_UART_GetError(huart);

        rx_index = 0u;

        if ((error & HAL_UART_ERROR_ORE) != 0u)
        {
            if (Analysis_StartByteReceive() != HAL_OK)
            {
                uart_restart_required = 1u;
            }
        }
    }
}

void Analysis_InputByte(uint8_t byte)
{
    if (rx_index == 0u)
    {
        if (byte == PACKET_HEADER)
        {
            rx_build_packet[0] = byte;
            rx_index = 1u;
        }

        return;
    }

    rx_build_packet[rx_index] = byte;
    rx_index++;

    if (rx_index < ANALYSIS_PACKET_LENGTH)
    {
        return;
    }

    for (uint32_t i = 0; i < ANALYSIS_PACKET_LENGTH; i++)
    {
        rx_ready_packet[i] = rx_build_packet[i];
    }

    packet_ready = 1u;

    /* If a damaged frame ends with a new header, keep that byte as the
     * beginning of the next frame so the stream can resynchronize quickly. */
    if (byte == PACKET_HEADER)
    {
        rx_build_packet[0] = byte;
        rx_index = 1u;
    }
    else
    {
        rx_index = 0u;
    }
}

void Analysis_Update(void)
{
    uint8_t packet[ANALYSIS_PACKET_LENGTH];
    uint8_t has_packet = 0u;
    uint32_t primask = __get_PRIMASK();

    Analysis_RestartUartIfRequired();

    __disable_irq();

    if (packet_ready != 0u)
    {
        for (uint32_t i = 0; i < ANALYSIS_PACKET_LENGTH; i++)
        {
            packet[i] = rx_ready_packet[i];
        }

        packet_ready = 0u;
        has_packet = 1u;
    }

    if (primask == 0u)
    {
        __enable_irq();
    }

    if (has_packet == 0u)
    {
        return;
    }

    if (Analysis_CheckPacket(packet) == 0u)
    {
        return;
    }

    Analysis_CalculateTarget(packet[1], packet[2], &current_target);
}

static HAL_StatusTypeDef Analysis_StartByteReceive(void)
{
    HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart6, &uart_rx_byte, 1u);

    if (status == HAL_OK)
    {
        uart_restart_required = 0u;
    }

    return status;
}

static void Analysis_RestartUartIfRequired(void)
{
    uint8_t restart_required;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    restart_required = uart_restart_required;
    uart_restart_required = 0u;

    if (primask == 0u)
    {
        __enable_irq();
    }

    if (restart_required == 0u)
    {
        return;
    }

    (void)HAL_UART_AbortReceive(&huart6);

    if (Analysis_StartByteReceive() != HAL_OK)
    {
        uart_restart_required = 1u;
    }
}

uint8_t Analysis_GetTargetRPM(Analysis_TargetRPM_t *target)
{
    if (target == NULL)
    {
        return 0u;
    }

    *target = current_target;
    return target->valid;
}

uint8_t Analysis_IsTargetTimeout(uint32_t now_ms)
{
    if (current_target.valid == 0u)
    {
        return 1u;
    }

    if ((now_ms - current_target.tick_ms) > ANALYSIS_TIMEOUT_MS)
    {
        return 1u;
    }

    return 0u;
}

static uint8_t Analysis_CRC8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0x00u;

    for (uint8_t i = 0u; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0u; bit < 8u; bit++)
        {
            if ((crc & 0x80u) != 0u)
            {
                crc = (uint8_t)((crc << 1u) ^ CRC8_POLY);
            }
            else
            {
                crc = (uint8_t)(crc << 1u);
            }
        }
    }

    return crc;
}

static uint8_t Analysis_CheckPacket(const uint8_t packet[ANALYSIS_PACKET_LENGTH])
{
    uint8_t mode = packet[1];
    uint8_t speed = packet[2];

    if (packet[0] != PACKET_HEADER)
    {
        return 0u;
    }

    if (packet[4] != PACKET_TAIL)
    {
        return 0u;
    }

    if (mode > 6u)
    {
        return 0u;
    }

    if (mode == 0u)
    {
        if (speed != 0u)
        {
            return 0u;
        }
    }
    else if ((speed < 1u) || (speed > 3u))
    {
        return 0u;
    }

    if (Analysis_CRC8(packet, 3u) != packet[3])
    {
        return 0u;
    }

    return 1u;
}

static void Analysis_CalculateTarget(uint8_t mode, uint8_t speed, Analysis_TargetRPM_t *target)
{
    static const float speed_rpm[4] =
    {
        0.0f,
        ANALYSIS_SPEED_1_RPM,
        ANALYSIS_SPEED_2_RPM,
        ANALYSIS_SPEED_3_RPM,
    };

    static const int8_t direction[7][4] =
    {
        { 0,  0,  0,  0},
        { 1,  1,  1,  1},
        {-1, -1, -1, -1},
        {-1,  1,  1, -1},
        { 1, -1, -1,  1},
        {-1,  1, -1,  1},
        { 1, -1,  1, -1},
    };

    float rpm = speed_rpm[speed];

    target->front_left_rpm = rpm * direction[mode][0];
    target->front_right_rpm = rpm * direction[mode][1];
    target->back_left_rpm = rpm * direction[mode][2];
    target->back_right_rpm = rpm * direction[mode][3];
    target->tick_ms = HAL_GetTick();
    target->valid = 1u;
}
