#include "stm32f4xx_hal.h"
#include "proximity.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_adc_ex.h"
#include <string.h>

static struct Uint16Array8 adc_dma_buffer = {0};
static ADC_HandleTypeDef *adc_handle = NULL;
static proximity_fn_cb user_callback = NULL;

void proximity_init(ADC_HandleTypeDef *hadc1, proximity_fn_cb callback)
{
    adc_handle = hadc1;
    user_callback = callback;
}

int proximity_start(void)
{
    return HAL_ADC_Start_DMA(adc_handle, (uint32_t *)&adc_dma_buffer, sizeof(struct Uint16Array8) / sizeof(uint16_t));
}

int proximity_stop(void)
{
    return HAL_ADC_Stop_DMA(adc_handle);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        if (user_callback)
            user_callback(&adc_dma_buffer);
    }
}