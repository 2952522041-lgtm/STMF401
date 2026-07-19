#include "UARTMODEL.h"
#include "usart.h"

#define UARTMODEL_MAX_PAYLOAD_LENGTH 32u
#define UARTMODEL_RX_DMA_BUFFER_LENGTH 64u
#define UARTMODEL_TARGET_PAYLOAD_LENGTH 8u
#define UARTMODEL_TELEMETRY_PAYLOAD_LENGTH 32u
#define UARTMODEL_FRAME_OVERHEAD 5u
#define UARTMODEL_TX_TIMEOUT_MS 10u

typedef enum
{
    UARTMODEL_RX_HEADER_1 = 0,
    UARTMODEL_RX_HEADER_2,
    UARTMODEL_RX_TYPE,
    UARTMODEL_RX_LENGTH,
    UARTMODEL_RX_PAYLOAD,
    UARTMODEL_RX_CHECKSUM,
} UARTMODEL_RxState_t;

static uint8_t uart_rx_dma_buffer[UARTMODEL_RX_DMA_BUFFER_LENGTH];
static UARTMODEL_RxState_t uart_rx_state = UARTMODEL_RX_HEADER_1;
static uint8_t uart_rx_type = 0u;
static uint8_t uart_rx_length = 0u;
static uint8_t uart_rx_index = 0u;
static uint8_t uart_rx_checksum = 0u;
static uint8_t uart_rx_payload[UARTMODEL_MAX_PAYLOAD_LENGTH];
static volatile int16_t uart_target_rpm_x10[UARTMODEL_MOTOR_NUM] = {0};
static volatile uint8_t uart_target_ready = 0u;

static HAL_StatusTypeDef UARTMODEL_StartReceive(void);

static int16_t UARTMODEL_FloatToInt16(float value)
{
    float scaled = value * 10.0f;

    if (scaled > 32767.0f)
    {
        scaled = 32767.0f;
    }
    else if (scaled < -32768.0f)
    {
        scaled = -32768.0f;
    }

    return (int16_t)scaled;
}

static void UARTMODEL_PutInt16(uint8_t *data, int16_t value)
{
    data[0] = (uint8_t)((uint16_t)value & 0xFFu);
    data[1] = (uint8_t)(((uint16_t)value >> 8u) & 0xFFu);
}

static void UARTMODEL_PutInt32(uint8_t *data, int32_t value)
{
    data[0] = (uint8_t)((uint32_t)value & 0xFFu);
    data[1] = (uint8_t)(((uint32_t)value >> 8u) & 0xFFu);
    data[2] = (uint8_t)(((uint32_t)value >> 16u) & 0xFFu);
    data[3] = (uint8_t)(((uint32_t)value >> 24u) & 0xFFu);
}

static void UARTMODEL_ProcessFrame(void)
{
    if ((uart_rx_type == UARTMODEL_CMD_SET_TARGET_RPM) && (uart_rx_length == UARTMODEL_TARGET_PAYLOAD_LENGTH))
    {
        for (uint32_t i = 0; i < UARTMODEL_MOTOR_NUM; i++)
        {
            uart_target_rpm_x10[i] = (int16_t)((uint16_t)uart_rx_payload[i * 2u] |
                                                  ((uint16_t)uart_rx_payload[i * 2u + 1u] << 8u));
        }

        uart_target_ready = 1u;
    }
    else if ((uart_rx_type == UARTMODEL_CMD_STOP) && (uart_rx_length == 0u))
    {
        for (uint32_t i = 0; i < UARTMODEL_MOTOR_NUM; i++)
        {
            uart_target_rpm_x10[i] = 0;
        }

        uart_target_ready = 1u;
    }
}

static void UARTMODEL_ParseByte(uint8_t byte)
{
    switch (uart_rx_state)
    {
        case UARTMODEL_RX_HEADER_1:
            if (byte == UARTMODEL_FRAME_HEADER_1)
            {
                uart_rx_state = UARTMODEL_RX_HEADER_2;
            }
            break;

        case UARTMODEL_RX_HEADER_2:
            uart_rx_state = (byte == UARTMODEL_FRAME_HEADER_2) ? UARTMODEL_RX_TYPE : UARTMODEL_RX_HEADER_1;
            break;

        case UARTMODEL_RX_TYPE:
            uart_rx_type = byte;
            uart_rx_checksum = byte;
            uart_rx_state = UARTMODEL_RX_LENGTH;
            break;

        case UARTMODEL_RX_LENGTH:
            uart_rx_length = byte;
            uart_rx_checksum ^= byte;
            uart_rx_index = 0u;

            if (uart_rx_length > UARTMODEL_MAX_PAYLOAD_LENGTH)
            {
                uart_rx_state = UARTMODEL_RX_HEADER_1;
            }
            else if (uart_rx_length == 0u)
            {
                uart_rx_state = UARTMODEL_RX_CHECKSUM;
            }
            else
            {
                uart_rx_state = UARTMODEL_RX_PAYLOAD;
            }
            break;

        case UARTMODEL_RX_PAYLOAD:
            uart_rx_payload[uart_rx_index++] = byte;
            uart_rx_checksum ^= byte;

            if (uart_rx_index >= uart_rx_length)
            {
                uart_rx_state = UARTMODEL_RX_CHECKSUM;
            }
            break;

        case UARTMODEL_RX_CHECKSUM:
            if (byte == uart_rx_checksum)
            {
                UARTMODEL_ProcessFrame();
            }

            uart_rx_state = UARTMODEL_RX_HEADER_1;
            break;

        default:
            uart_rx_state = UARTMODEL_RX_HEADER_1;
            break;
    }
}

static void UARTMODEL_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART6)
    {
        for (uint16_t i = 0u; i < size; i++)
        {
            UARTMODEL_ParseByte(uart_rx_dma_buffer[i]);
        }

        (void)UARTMODEL_StartReceive();
    }
}

static void UARTMODEL_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        uart_rx_state = UARTMODEL_RX_HEADER_1;
        (void)UARTMODEL_StartReceive();
    }
}

static HAL_StatusTypeDef UARTMODEL_StartReceive(void)
{
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uart_rx_dma_buffer, UARTMODEL_RX_DMA_BUFFER_LENGTH);

    if (status == HAL_OK)
    {
        /* An IDLE or full-buffer event is sufficient; a half-transfer callback
         * would split and restart a normal-mode DMA reception unnecessarily. */
        __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
    }

    return status;
}

void UARTMODEL_Init(void)
{
    if (HAL_UART_RegisterRxEventCallback(&huart6, UARTMODEL_RxEventCallback) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_UART_RegisterCallback(&huart6, HAL_UART_ERROR_CB_ID, UARTMODEL_ErrorCallback) != HAL_OK)
    {
        Error_Handler();
    }

    if (UARTMODEL_StartReceive() != HAL_OK)
    {
        Error_Handler();
    }
}

uint8_t UARTMODEL_GetTargetRPM(float target_rpm[UARTMODEL_MOTOR_NUM])
{
    if ((target_rpm == NULL) || (uart_target_ready == 0u))
    {
        return 0u;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    for (uint32_t i = 0; i < UARTMODEL_MOTOR_NUM; i++)
    {
        target_rpm[i] = (float)uart_target_rpm_x10[i] / 10.0f;
    }

    uart_target_ready = 0u;

    if (primask == 0u)
    {
        __enable_irq();
    }

    return 1u;
}

HAL_StatusTypeDef UARTMODEL_SendTelemetry(const float target_rpm[UARTMODEL_MOTOR_NUM],
                                          const float measured_rpm[UARTMODEL_MOTOR_NUM],
                                          const int32_t encoder_count[UARTMODEL_MOTOR_NUM])
{
    if ((target_rpm == NULL) || (measured_rpm == NULL) || (encoder_count == NULL))
    {
        return HAL_ERROR;
    }

    uint8_t frame[UARTMODEL_FRAME_OVERHEAD + UARTMODEL_TELEMETRY_PAYLOAD_LENGTH];
    uint8_t *payload = &frame[4];

    frame[0] = UARTMODEL_FRAME_HEADER_1;
    frame[1] = UARTMODEL_FRAME_HEADER_2;
    frame[2] = UARTMODEL_MSG_TELEMETRY;
    frame[3] = UARTMODEL_TELEMETRY_PAYLOAD_LENGTH;

    for (uint32_t i = 0; i < UARTMODEL_MOTOR_NUM; i++)
    {
        UARTMODEL_PutInt16(&payload[i * 2u], UARTMODEL_FloatToInt16(target_rpm[i]));
        UARTMODEL_PutInt16(&payload[8u + i * 2u], UARTMODEL_FloatToInt16(measured_rpm[i]));
        UARTMODEL_PutInt32(&payload[16u + i * 4u], encoder_count[i]);
    }

    uint8_t checksum = 0u;
    for (uint32_t i = 2u; i < 4u + UARTMODEL_TELEMETRY_PAYLOAD_LENGTH; i++)
    {
        checksum ^= frame[i];
    }
    frame[4u + UARTMODEL_TELEMETRY_PAYLOAD_LENGTH] = checksum;

    return HAL_UART_Transmit(&huart6, frame, sizeof(frame), UARTMODEL_TX_TIMEOUT_MS);
}
