#include "app_task.h"
#include "analysis.h"
#include "encode.h"
#include "Motor.h"
#include "tb6612.h"
#include "tim.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "dsp/controller_functions.h"

float SPEED_PID_KP = 0.5f;
float SPEED_PID_KI = 0.0f;
float SPEED_PID_KD = 0.0f;

#define SPEED_PID_OUTPUT_LIMIT ((float)tb6612_max_rpm)
#define SPEED_SAMPLE_TASK_STACK_SIZE 256u
#define SPEED_SAMPLE_TASK_PRIORITY (tskIDLE_PRIORITY + 3u)
#define SPEED_PID_TASK_STACK_SIZE 384u
#define SPEED_PID_TASK_PRIORITY (tskIDLE_PRIORITY + 2u)
#define RECEIVE_TARGET_RPM_TASK_STACK_SIZE 256u
#define RECEIVE_TARGET_RPM_TASK_PRIORITY (tskIDLE_PRIORITY + 1u)
#define SPEED_SAMPLE_QUEUE_LENGTH 1u
#define SPEED_CONTROL_FREQUENCY_HZ 100.0f

typedef struct
{
    float target_rpm[MOTOR_NUM];
    float measured_rpm[MOTOR_NUM];
} speed_sample_t;

static void speed_sample_task(void *pvParameters);
static void speed_pid_task(void *pvParameters);
static void receive_target_rpm_task(void *pvParameters);
static void APP_TIM10PeriodElapsedCallback(TIM_HandleTypeDef *htim);

static SemaphoreHandle_t speed_tick_sem = NULL;
static QueueHandle_t speed_sample_queue = NULL;
static arm_pid_instance_f32 speed_pid[MOTOR_NUM];
static float last_target_rpm[MOTOR_NUM] = {0.0f};

static float SpeedPID_Limit(float value)
{
    if (value > SPEED_PID_OUTPUT_LIMIT)
    {
        value = SPEED_PID_OUTPUT_LIMIT;
    }
    else if (value < -SPEED_PID_OUTPUT_LIMIT)
    {
        value = -SPEED_PID_OUTPUT_LIMIT;
    }

    return value;
}

void APP_FREERTOS_Init(void)
{
    speed_tick_sem = xSemaphoreCreateBinary();
    speed_sample_queue = xQueueCreate(SPEED_SAMPLE_QUEUE_LENGTH, sizeof(speed_sample_t));

    if ((speed_tick_sem == NULL) || (speed_sample_queue == NULL))
    {
        Error_Handler();
    }

    for (uint32_t i = 0; i < MOTOR_NUM; i++)
    {
        speed_pid[i].Kp = SPEED_PID_KP;
        speed_pid[i].Ki = SPEED_PID_KI / SPEED_CONTROL_FREQUENCY_HZ;
        speed_pid[i].Kd = SPEED_PID_KD * SPEED_CONTROL_FREQUENCY_HZ;
        arm_pid_init_f32(&speed_pid[i], 1);
    }

    if (xTaskCreate(speed_sample_task,
                    "SpeedSampleTask",
                    SPEED_SAMPLE_TASK_STACK_SIZE,
                    NULL,
                    SPEED_SAMPLE_TASK_PRIORITY,
                    NULL) != pdPASS)
    {
        Error_Handler();
    }

    if (xTaskCreate(speed_pid_task,
                    "SpeedPIDTask",
                    SPEED_PID_TASK_STACK_SIZE,
                    NULL,
                    SPEED_PID_TASK_PRIORITY,
                    NULL) != pdPASS)
    {
        Error_Handler();
    }

    if (xTaskCreate(receive_target_rpm_task,
                    "ReceiveTargetRPMTask",
                    RECEIVE_TARGET_RPM_TASK_STACK_SIZE,
                    NULL,
                    RECEIVE_TARGET_RPM_TASK_PRIORITY,
                    NULL) != pdPASS)
    {
        Error_Handler();
    }
}

void App_Timer100HZISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ((speed_tick_sem != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
    {
        xSemaphoreGiveFromISR(speed_tick_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void vApplicationMallocFailedHook(void)
{
    Error_Handler();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    Error_Handler();
}

static void APP_TIM10PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim10)
    {
        App_Timer100HZISR();
    }
}

static void speed_sample_task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_tick = xTaskGetTickCount();

    for (;;)
    {
        if (xSemaphoreTake(speed_tick_sem, portMAX_DELAY) == pdTRUE)
        {
            speed_sample_t sample;
            TickType_t now_tick = xTaskGetTickCount();
            TickType_t elapsed_tick = now_tick - last_tick;
            float dt_sec;

            if (elapsed_tick == 0u)
            {
                dt_sec = 1.0f / SPEED_CONTROL_FREQUENCY_HZ;
            }
            else
            {
                dt_sec = (float)elapsed_tick / configTICK_RATE_HZ;
            }

            last_tick = now_tick;
            Encoder_Update(dt_sec);
            Motor_GetAllTargetRPM(sample.target_rpm);

            for (uint32_t i = 0; i < MOTOR_NUM; i++)
            {
                sample.measured_rpm[i] = Encoder_GetRPM((Encoder_ID_t)i);
            }

            xQueueOverwrite(speed_sample_queue, &sample);
        }
    }
}

static void speed_pid_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        speed_sample_t sample;

        if (xQueueReceive(speed_sample_queue, &sample, portMAX_DELAY) == pdTRUE)
        {
            float output_rpm[MOTOR_NUM] = {0.0f};

            for (uint32_t i = 0; i < MOTOR_NUM; i++)
            {
                float error = sample.target_rpm[i] - sample.measured_rpm[i];

                if (sample.target_rpm[i] == 0.0f)
                {
                    arm_pid_reset_f32(&speed_pid[i]);
                }
                else
                {
                    if ((last_target_rpm[i] * sample.target_rpm[i]) < 0.0f)
                    {
                        arm_pid_reset_f32(&speed_pid[i]);
                    }

                    output_rpm[i] = arm_pid_f32(&speed_pid[i], error);
                }

                output_rpm[i] = SpeedPID_Limit(output_rpm[i]);
                last_target_rpm[i] = sample.target_rpm[i];
            }

            Motor_SetAllRPM(output_rpm[MOTOR_FRONT_LEFT],
                            output_rpm[MOTOR_FRONT_RIGHT],
                            output_rpm[MOTOR_BACK_LEFT],
                            output_rpm[MOTOR_BACK_RIGHT]);
        }
    }
}

static void receive_target_rpm_task(void *pvParameters)
{
    Analysis_TargetRPM_t target;
    TickType_t last_wake_time;

    (void)pvParameters;

    Motor_SetAllTargetRPM(0.0f, 0.0f, 0.0f, 0.0f);
    last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        Analysis_Update();

        if ((Analysis_GetTargetRPM(&target) != 0u) &&
            (Analysis_IsTargetTimeout(HAL_GetTick()) == 0u))
        {
            Motor_SetAllTargetRPM(target.front_left_rpm,
                                  target.front_right_rpm,
                                  target.back_left_rpm,
                                  target.back_right_rpm);
        }
        else
        {
            Motor_SetAllTargetRPM(0.0f, 0.0f, 0.0f, 0.0f);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10u));
    }
}

void User_Init(void)
{
    Motor_Init();
    Encoder_Init();
    Analysis_Init();
    Analysis_StartUartReceive();

    if (HAL_TIM_RegisterCallback(&htim10, HAL_TIM_PERIOD_ELAPSED_CB_ID, APP_TIM10PeriodElapsedCallback) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_Base_Start_IT(&htim10) != HAL_OK)
    {
        Error_Handler();
    }
}
