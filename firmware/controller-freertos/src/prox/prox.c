#include "prox.h"
#include "main.h"
#include "cmsis_os.h"

#include "flatcc/flatcc.h"
#include "sensors_builder.h"

struct port_pin_pair
{
    GPIO_TypeDef *port;
    uint16_t pin;
};

enum pulse_state
{
    CYCLE0_OFF,
    CYCLE0_ON,
    CYCLE1_OFF,
    CYCLE1_ON,
    CYCLE2_OFF,
    CYCLE2_ON,
    CYCLE3_OFF,
    CYCLE3_ON,
};

TIM_HandleTypeDef *timer_handle = NULL;
ADC_HandleTypeDef *adc_handle = NULL;
TaskHandle_t proximity_task_handle = NULL;
int pulse_state = 0;

/** Buffer filled by DMA containing the ambient and absolute distance values.
 * Structured in pairs: [0:1] contain pairs for sensor PROX0 IR ON, IR OFF and so on.
 */
volatile uint16_t adc_buffer[16];

#define MAX_PROXIMITY_SAMPLES 10
static osMutexId_t prox_data_mutex;
static FripuckProtocol_Sensors_ProximityData_t proximity_buffer[MAX_PROXIMITY_SAMPLES] = {0};
static uint32_t write_pointer = 0;
static uint32_t read_pointer = 0;

void proximity_timer_elapsed_cb(TIM_HandleTypeDef *htim);
void proximity_timer_channel1_disable_ir_cb(TIM_HandleTypeDef *htim);

void ProximityTask(void *argument)
{
    while (1)
    {
        // Block indefinitely until the ADC DMA interrupt wakes this task
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        FripuckProtocol_Sensors_ProximityData_t *p = &proximity_buffer[write_pointer % MAX_PROXIMITY_SAMPLES];

        osMutexAcquire(prox_data_mutex, osWaitForever);
        // Copy proximity values
        p->proximity.a0 = adc_buffer[0 * 2 + 0];
        p->proximity.a1 = adc_buffer[1 * 2 + 0];
        p->proximity.a2 = adc_buffer[2 * 2 + 0];
        p->proximity.a3 = adc_buffer[3 * 2 + 0];
        p->proximity.a4 = adc_buffer[4 * 2 + 0];
        p->proximity.a5 = adc_buffer[5 * 2 + 0];
        p->proximity.a6 = adc_buffer[6 * 2 + 0];
        p->proximity.a7 = adc_buffer[7 * 2 + 0];

        // Copy ambient values
        p->ambient_light.a0 = adc_buffer[0 * 2 + 1];
        p->ambient_light.a1 = adc_buffer[1 * 2 + 1];
        p->ambient_light.a2 = adc_buffer[2 * 2 + 1];
        p->ambient_light.a3 = adc_buffer[3 * 2 + 1];
        p->ambient_light.a4 = adc_buffer[4 * 2 + 1];
        p->ambient_light.a5 = adc_buffer[5 * 2 + 1];
        p->ambient_light.a6 = adc_buffer[6 * 2 + 1];
        p->ambient_light.a7 = adc_buffer[7 * 2 + 1];

        // Set the time offset of that specific proximity packet
        p->timestamp_offset = (uint16_t)pdTICKS_TO_MS(HAL_GetTick());
        p->padding = 0;
        osMutexRelease(prox_data_mutex);

        write_pointer++;
    }
}

void pack_prox_to_vector(flatcc_builder_t *builder)
{
    osMutexAcquire(prox_data_mutex, osWaitForever);
    uint32_t current_read = read_pointer;
    uint32_t current_write = write_pointer;
    uint32_t count = current_write - current_read;

    // if we read to slowly, maybe there are more than that missing.
    if (count > MAX_PROXIMITY_SAMPLES)
    {
        count = MAX_PROXIMITY_SAMPLES;
        current_read = current_write - MAX_PROXIMITY_SAMPLES;
    }

    if (count <= 0)
    {
        osMutexRelease(prox_data_mutex);
        return;
    }

    FripuckProtocol_Sensors_ProximityData_vec_start(builder);

    uint32_t start_idx = current_read % MAX_PROXIMITY_SAMPLES;
    uint32_t end_idx = current_write % MAX_PROXIMITY_SAMPLES;

    if (start_idx < end_idx)
    {
        // Linear case: Data is in one continuous block
        FripuckProtocol_Sensors_ProximityData_vec_append(builder,
                                                         (const FripuckProtocol_Sensors_ProximityData_t *)&proximity_buffer[start_idx], end_idx - start_idx);
    }
    else
    {
        // Wrapped case: Data is split across the array boundary
        // Part A: From read_pointer to the very end of the array
        FripuckProtocol_Sensors_ProximityData_vec_append(builder,
                                                         (const FripuckProtocol_Sensors_ProximityData_t *)&proximity_buffer[start_idx], MAX_PROXIMITY_SAMPLES - start_idx);
        // Part B: From the start of the array to the write_pointer
        FripuckProtocol_Sensors_ProximityData_vec_append(builder,
                                                         (const FripuckProtocol_Sensors_ProximityData_t *)&proximity_buffer[0], end_idx);
    }

    // Add what was added
    FripuckProtocol_Sensors_SensorBatch_proximity_add(builder, FripuckProtocol_Sensors_ProximityData_vec_end(builder));

    read_pointer = current_write;
    osMutexRelease(prox_data_mutex);
}

void proximity_start(TIM_HandleTypeDef *tim5_handle, ADC_HandleTypeDef *adc1_handle)
{
    timer_handle = tim5_handle;
    adc_handle = adc1_handle;
    pulse_state = 0;

    HAL_TIM_RegisterCallback(timer_handle, HAL_TIM_PERIOD_ELAPSED_CB_ID, proximity_timer_elapsed_cb);
    HAL_TIM_RegisterCallback(timer_handle, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, proximity_timer_channel1_disable_ir_cb);

    // Create a mutex so we can't read/modify the data while it is being updated
    static const osMutexAttr_t mutex_attributes = {"prox_data_mutex", osMutexRecursive | osMutexPrioInherit, NULL, 0};
    prox_data_mutex = osMutexNew(&mutex_attributes);

    // Start ADC with DMA
    HAL_ADC_Start_DMA(adc_handle, (uint32_t *)adc_buffer, 16);

    // Start TIM5 (Base, CH1 Interrupt, and CH2 PWM for ADC Trigger)
    HAL_TIM_Base_Start_IT(timer_handle);
    HAL_TIM_OC_Start_IT(timer_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(timer_handle, TIM_CHANNEL_2);

    BaseType_t status = xTaskCreate(ProximityTask, "proximity_task", 256, NULL, osPriorityNormal, &proximity_task_handle);
    if (status != pdPASS)
    {
        while (1)
            ; // Task creation failed (likely out of Heap memory)
    }
}

void proximity_stop()
{
    HAL_TIM_Base_Stop_IT(timer_handle);
    HAL_TIM_OC_Stop_IT(timer_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(timer_handle, TIM_CHANNEL_2);
    HAL_ADC_Stop_DMA(adc_handle);

    vTaskDelete(proximity_task_handle);
}

/* Called when timer 5 elapsed.
Since we have a rate of 1 tick/µs and an ARR of 1250, this is triggered at a rate of 800 Hz.
Once the time has elapsed, the following happens after...

- 0µs: It immediately enables turns on the correct IR light if needed (on odd states)
- 260µs: Channel 2 gets activated which triggers ADC to read the value
- 300µs: Channel 1 gets activated which triggers the callback `HAL_TIM_OC_DelayElapsedCallback` and turns off the IR lights.

The data is read twice (two cycles) for each pin, but in the first cycle, the ir light is on, and on the second cycle, the ir light is off (reads the ambient light).
 */
void proximity_timer_elapsed_cb(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        // Only turn LEDs on during odd states (Reflected). Even states are Ambient (leave off).
        switch (pulse_state)
        {
        case CYCLE0_ON:
            HAL_GPIO_WritePin(PROX0_GPIO_Port, PROX0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(PROX4_GPIO_Port, PROX4_Pin, GPIO_PIN_SET);
            break;
        case CYCLE1_ON:
            HAL_GPIO_WritePin(PROX1_GPIO_Port, PROX1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(PROX5_GPIO_Port, PROX5_Pin, GPIO_PIN_SET);
            break;
        case CYCLE2_ON:
            HAL_GPIO_WritePin(GPIOC, PROX2_Pin | PROX6_Pin, GPIO_PIN_SET); // both pins on gpioc

            break;
        case CYCLE3_ON:
            HAL_GPIO_WritePin(GPIOC, PROX3_Pin | PROX7_Pin, GPIO_PIN_SET); // both pins on gpioc

            break;
        default:
            break; // Ambient states (0, 2, 4, 6): Do nothing
        }
    }
}

// Gets called 300µs after TIM5 has elapsed to turn off all ir lights.
void proximity_timer_channel1_disable_ir_cb(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        // Disable all IR lights
        HAL_GPIO_WritePin(GPIOB, PROX4_Pin | PROX5_Pin, GPIO_PIN_RESET);                                     // B
        HAL_GPIO_WritePin(GPIOC, PROX0_Pin | PROX2_Pin | PROX3_Pin | PROX6_Pin | PROX7_Pin, GPIO_PIN_RESET); // C

        // Move to next state
        pulse_state = (pulse_state + 1) % 8;
    }
}

// Once the program has done all 16 readings, this gets called
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == adc_handle->Instance && proximity_task_handle != NULL)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Notify the FreeRTOS task that 16 samples are ready in the DMA buffer
        vTaskNotifyGiveFromISR(proximity_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}