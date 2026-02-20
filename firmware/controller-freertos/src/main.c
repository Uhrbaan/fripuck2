#include "main.h"
#include "core/hardware_init.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

#include "leds.h"
#include "motors.h"

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
void StartDefaultTask(void *argument)
{
    while (1)
    {
        // --- TEST 1: Forward Movement ---
        motor_set_direction(MOTOR_LEFT, false);
        motor_set_direction(MOTOR_RIGHT, false);

        // Slow
        motor_set_speed(MOTOR_LEFT, 200);
        motor_set_speed(MOTOR_RIGHT, 200);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Fast
        motor_set_speed(MOTOR_LEFT, 800);
        motor_set_speed(MOTOR_RIGHT, 800);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // --- TEST 2: Stop ---
        motor_set_speed(MOTOR_LEFT, 0);
        motor_set_speed(MOTOR_RIGHT, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // --- TEST 3: Reverse Movement ---
        motor_set_direction(MOTOR_LEFT, true);
        motor_set_direction(MOTOR_RIGHT, true);

        motor_set_speed(MOTOR_LEFT, 400);
        motor_set_speed(MOTOR_RIGHT, 400);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // --- TEST 4: Async Speeds (Turning) ---
        motor_set_speed(MOTOR_LEFT, 800);
        motor_set_speed(MOTOR_RIGHT, 200);
        vTaskDelay(pdMS_TO_TICKS(3000));

        // Final Stop before repeating
        motor_set_speed(MOTOR_LEFT, 0);
        motor_set_speed(MOTOR_RIGHT, 0);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();

    motors_init();

    /* Init scheduler */
    osKernelInitialize();
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    osKernelStart();
    while (1)
    {
    }
}