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
#include "dsp/controller_functions.h"

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
    float target_rpm[MOTOR_NUM];
    float measured_rpm[MOTOR_NUM];
    int32_t encoder_count[MOTOR_NUM];
} SpeedSample_t;

static SemaphoreHandle_t speed_tick_sem = NULL;
static QueueHandle_t telemetry_queue = NULL;
static arm_pid_instance_f32 speed_pid[MOTOR_NUM] = {0};
static float previous_target_rpm[MOTOR_NUM] = {0.0f};

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

static void SpeedPID_Reset(uint32_t motor)
{
    arm_pid_reset_f32(&speed_pid[motor]);
    previous_target_rpm[motor] = 0.0f;
}

static float SpeedPID_Update(uint32_t motor, float target, float measured)
{
    if (target == 0.0f)
    {
        SpeedPID_Reset(motor);
        return 0.0f;
    }

    if ((previous_target_rpm[motor] * target) < 0.0f)
    {
        SpeedPID_Reset(motor);
    }

    float error = target - measured;
    float output_rpm = arm_pid_f32(&speed_pid[motor], error);
    previous_target_rpm[motor] = target;

    return App_Limit(output_rpm, tb6612_max_rpm);
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

    for (uint32_t i = 0; i < MOTOR_NUM; i++)
    {
        speed_pid[i].Kp = SPEED_PID_KP;
        speed_pid[i].Ki = SPEED_PID_KI / SPEED_CONTROL_FREQUENCY;
        speed_pid[i].Kd = SPEED_PID_KD * SPEED_CONTROL_FREQUENCY;
        arm_pid_init_f32(&speed_pid[i], 1);
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

                float output_rpm = SpeedPID_Update(i, sample.target_rpm[i], sample.measured_rpm[i]);
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
