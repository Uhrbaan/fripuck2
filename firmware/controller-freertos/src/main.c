#include "core/main.h"
#include "core/hardware_init.h"
#include "cmsis_os.h"

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
    for (;;)
    {
        HAL_GPIO_TogglePin(LED_BODY_GPIO_Port, LED_BODY_Pin);
        vTaskDelay(pdMS_TO_TICKS(500));
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

    /* Init scheduler */
    osKernelInitialize();
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    osKernelStart();
    while (1)
    {
    }
}