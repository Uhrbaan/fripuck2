#include "main.h"
#include "selector.h"

uint8_t get_selector() {
    uint8_t selector = HAL_GPIO_ReadPin(SEL_0_GPIO_Port, SEL_0_Pin) + 2 * HAL_GPIO_ReadPin(SEL_1_GPIO_Port, SEL_1_Pin) +
                       4 * HAL_GPIO_ReadPin(SEL_2_GPIO_Port, SEL_2_Pin) +
                       8 * HAL_GPIO_ReadPin(SEL_3_GPIO_Port, SEL_3_Pin);
    return selector;
}
