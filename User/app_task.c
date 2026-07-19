#include "app_task.h"
#include "Motor.h"
#include "UARTMODEL.h"
#include "encode.h"
#include "tb6612.h"
#include "tim.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#define SPEED_CONTROL_FREQUENCY 100.0f
#define SPEED_CONTROL_DT_SEC (1.0f / SPEED_CONTROL_FREQUENCY)
#define SPEED_PID_KP 0.5f
#define SPEED_PID_KI 0.0f
#define SPEED_PID_KD 0.0f
#define SPEED_CONTROL_TASK_STACK_SIZE 384u
#define SPEED_CONTROL_TASK_PRIORITY (tskIDLE_PRIORITY + 3u)
#define UART_TELEMETRY_TASK_STACK_SIZE 256u
#define UART_TELEMETRY_TASK_PRIORITY (tskIDLE_PRIORITY + 1u)
#define TELEMETRY_QUEUE_LENGTH 1u

typedef struct
{
    float integral;
    float previous_error;
    float previous_target;
} SpeedPID_t;

typedef struct
{
    float target_rpm[MOTOR_NUM];
    float measured_rpm[MOTOR_NUM];
    int32_t encoder_count[MOTOR_NUM];
} SpeedSample_t;

static SemaphoreHandle_t speed_tick_sem = NULL;
static QueueHandle_t telemetry_queue = NULL;
static SpeedPID_t speed_pid[MOTOR_NUM] = {0};

static void SpeedControlTask(void *pvParameters);
static void UARTTelemetryTask(void *pvParameters);
static void APP_TIM10PeriodElapsedCallback(TIM_HandleTypeDef *htim);

static float App_Limit(float value, float limit)
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

static void SpeedPID_Reset(SpeedPID_t *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_target = 0.0f;
}

static float SpeedPID_Update(SpeedPID_t *pid, float target, float measured)
{
    if (target == 0.0f)
    {
        SpeedPID_Reset(pid);
        return 0.0f;
    }

    if ((pid->previous_target * target) < 0.0f)
    {
        SpeedPID_Reset(pid);
    }

    float error = target - measured;
    pid->integral += error * SPEED_CONTROL_DT_SEC;
    pid->integral = App_Limit(pid->integral, tb6612_max_rpm);

    float derivative = (error - pid->previous_error) / SPEED_CONTROL_DT_SEC;
    float correction = SPEED_PID_KP * error + SPEED_PID_KI * pid->integral + SPEED_PID_KD * derivative;

    pid->previous_error = error;
    pid->previous_target = target;

    return App_Limit(target + correction, tb6612_max_rpm);
}

void User_Init(void)
{
    Motor_Init();
    Encoder_Init();
    UARTMODEL_Init();

    if (HAL_TIM_RegisterCallback(&htim10, HAL_TIM_PERIOD_ELAPSED_CB_ID, APP_TIM10PeriodElapsedCallback) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_Base_Start_IT(&htim10) != HAL_OK)
    {
        Error_Handler();
    }
}

void APP_FREERTOS_Init(void)
{
    speed_tick_sem = xSemaphoreCreateBinary();
    telemetry_queue = xQueueCreate(TELEMETRY_QUEUE_LENGTH, sizeof(SpeedSample_t));

    if ((speed_tick_sem == NULL) || (telemetry_queue == NULL))
    {
        Error_Handler();
    }

    if (xTaskCreate(SpeedControlTask,
                    "SpeedControl",
                    SPEED_CONTROL_TASK_STACK_SIZE,
                    NULL,
                    SPEED_CONTROL_TASK_PRIORITY,
                    NULL) != pdPASS)
    {
        Error_Handler();
    }

    if (xTaskCreate(UARTTelemetryTask,
                    "UARTTelemetry",
                    UART_TELEMETRY_TASK_STACK_SIZE,
                    NULL,
                    UART_TELEMETRY_TASK_PRIORITY,
                    NULL) != pdPASS)
    {
        Error_Handler();
    }
}

void App_Timer100HZISR(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if ((speed_tick_sem != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
    {
        xSemaphoreGiveFromISR(speed_tick_sem, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

static void APP_TIM10PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10)
    {
        App_Timer100HZISR();
    }
}

static void SpeedControlTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        if (xSemaphoreTake(speed_tick_sem, portMAX_DELAY) == pdTRUE)
        {
            float uart_target[MOTOR_NUM];
            if (UARTMODEL_GetTargetRPM(uart_target) != 0u)
            {
                Motor_SetAllTargetRPM(uart_target[MOTOR_FRONT_LEFT],
                                      uart_target[MOTOR_FRONT_RIGHT],
                                      uart_target[MOTOR_BACK_LEFT],
                                      uart_target[MOTOR_BACK_RIGHT]);
            }

            Encoder_Update(SPEED_CONTROL_DT_SEC);

            SpeedSample_t sample;
            for (uint32_t i = 0; i < MOTOR_NUM; i++)
            {
                sample.target_rpm[i] = Motor_GetTargetRPM((Motor_ID_t)i);
                sample.measured_rpm[i] = Encoder_GetRPM((Encoder_ID_t)i);
                sample.encoder_count[i] = Encoder_GetCount((Encoder_ID_t)i);

                float output_rpm = SpeedPID_Update(&speed_pid[i], sample.target_rpm[i], sample.measured_rpm[i]);
                Motor_SetRPM((Motor_ID_t)i, output_rpm);
            }

            (void)xQueueOverwrite(telemetry_queue, &sample);
        }
    }
}

static void UARTTelemetryTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        SpeedSample_t sample;
        if (xQueueReceive(telemetry_queue, &sample, portMAX_DELAY) == pdTRUE)
        {
            (void)UARTMODEL_SendTelemetry(sample.target_rpm, sample.measured_rpm, sample.encoder_count);
        }
    }
}
