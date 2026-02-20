#include <inttypes.h>
#include "leds.h"
#include "main.h"

struct port_pin_pair
{
    GPIO_TypeDef *port;
    uint16_t pin;
};

static struct port_pin_pair led_port_pin_table[] = {
    [LED1] = {LED1_GPIO_Port, LED1_Pin},
    [LED3] = {LED3_GPIO_Port, LED3_Pin},
    [LED5] = {LED5_GPIO_Port, LED5_Pin},
    [LED7] = {LED7_GPIO_Port, LED7_Pin},
    [LED_BODY] = {LED_BODY_GPIO_Port, LED_BODY_Pin},
    [LED_FRONT] = {LED_FRONT_GPIO_Port, LED_BODY_Pin},
};

// FIXME: Front LED doesn't seem to work.
void set_led(led_name_t led_number, bool value)
{
    if (led_number < 0 || led_number >= NUM_LED)
        return;

    struct port_pin_pair pair = led_port_pin_table[led_number];
    HAL_GPIO_WritePin(pair.port, pair.pin, (value == true) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void toggle_led(led_name_t led_number)
{
    if (led_number < 0 || led_number >= NUM_LED)
        return;

    struct port_pin_pair pair = led_port_pin_table[led_number];
    HAL_GPIO_TogglePin(pair.port, pair.pin);
}

void clear_leds(void)
{
    for (int i = 0; i < sizeof(led_port_pin_table) / sizeof(led_port_pin_table[0]); i++)
    {
        set_led(i, false);
    }
}