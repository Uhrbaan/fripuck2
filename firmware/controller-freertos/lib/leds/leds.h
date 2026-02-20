#ifndef LEDS_H
#define LEDS_H

#include <stdbool.h>

#define RGB_MAX_INTENSITY 100 // percentage

// List of the RED LEDs present on the e-puck 2
typedef enum
{
    LED1,
    LED3,
    LED5,
    LED7,
    LED_BODY,
    LED_FRONT,
    NUM_LED,
} led_name_t;

/*! \brief Turn on/off the specified LED
 *
 * The e-puck2 has 4 red LEDs placed on front, right, back and left; these LEDs are directly controllable from the main processor (F407).
 * There are also 4 RGB LEDs placed at 45, 135, 225, 315 degrees; these LEDs are connected to the ESP32 and can be controlled thorugh SPI.
 * With this function, you can change the state of the 4 red LEDs, not the RGB LEDs.
 * \param led_number: LED1, LED3, LED5 or LED7 (LED1 is the front led, then continue clockwise)
 * \param value 0 (off), 1 (on) otherwise toggle the state
 * \warning if led_number is other than LED1-LED7, all leds are set to the indicated value.
 */
void set_led(led_name_t led_number, bool value);

void toggle_led(led_name_t led_number);

void clear_leds(void);

// TODO: RGB LEDs are handled by the ESP controller. This means that this controller won't implement them, since user commands are comming from the ESP chip anyway.

#endif
