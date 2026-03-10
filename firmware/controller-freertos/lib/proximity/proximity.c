#include "proximity.h"
#include "main.h"
#include <string.h>

extern ADC_HandleTypeDef *hadc1;

struct port_pin_pair
{
    GPIO_TypeDef *port;
    uint16_t pin;
};

static struct port_pin_pair proximity_port_pin_table[] = {
    [PROXIMITY_0] = {IR0_AN_GPIO_Port, IR0_AN_Pin},
    [PROXIMITY_1] = {IR1_AN_GPIO_Port, IR1_AN_Pin},
    [PROXIMITY_2] = {IR2_AN_GPIO_Port, IR2_AN_Pin},
    [PROXIMITY_3] = {IR3_AN_GPIO_Port, IR3_AN_Pin},
    [PROXIMITY_4] = {IR4_AN_GPIO_Port, IR4_AN_Pin},
    [PROXIMITY_5] = {IR5_AN_GPIO_Port, IR5_AN_Pin},
    [PROXIMITY_6] = {IR6_AN_GPIO_Port, IR6_AN_Pin},
    [PROXIMITY_7] = {IR7_AN_GPIO_Port, IR7_AN_Pin},
};

static uint16_t proximity_adc_dma_buffer[NUM_PROXIMITY] = {0};

void proximity_enable()
{
    HAL_ADC_Start_DMA(hadc1, (uint32_t *)proximity_adc_dma_buffer, NUM_PROXIMITY);
}

void proximity_disable()
{
    HAL_ADC_Stop_DMA(hadc1);
}

uint32_t proximity_get(enum proximity_name proximity_number)
{
    if (proximity_number < 0 || proximity_number >= NUM_PROXIMITY)
        return 0;

    return proximity_adc_dma_buffer[proximity_number];
}

void proximity_get_all(uint16_t *buffer)
{
    memcpy(buffer, proximity_adc_dma_buffer, sizeof(proximity_adc_dma_buffer));
}

/* FIXME: Implement clever tricks if measured to be mandatory
 * - turning ir lights off to read ambient light to substract it from total
 * - limiting power consumption by only turning the ir lights on when needed
 * - polling the ir lights at opposite positions to limit ir interference
 *
 * This work should only be done if deemed necessary.
 * If the impact of ambient light and interference is not significant enough, it shouldn't be changed.
 * If it has to be changed, one also has to change the polling to be discontinuous so the sensors can be polled individually.
 */
