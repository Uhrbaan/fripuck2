/*
Asercom protocol: https://www.gctronic.com/doc/index.php?title=Advanced_sercom_protocol
Asercom2 protocol: https://www.gctronic.com/doc/index.php?title=e-puck2_PC_side_development

// TODO: place a description of the protocol here.


This version of the asercom protocol aims to simplify and extend the already existing Asercom2 protocol. It will
simplify the protocol by removing checks for extensions or modules that are not in use for the epuck robots at the
university of Fribourg, which only use wifi and only come with the floor sensor extension. It is thus not portable (for
now at least).

This file replaces the absolutely massive 1000+ lines of code of a switch statement with a dispatch table.
*/
#include "../ir_remote.h"
#include "../usbcfg.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "behaviors.h"
#include "button.h"
#include "camera/dcmi_camera.h"
#include "chstreams.h"
#include "leds.h"
#include "sdio.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/ground.h"
#include "sensors/imu.h"
#include "sensors/proximity.h"
#include "serial_comm.h"
#include <a_d/advance_ad_scan/e_acc.h>
#include <a_d/advance_ad_scan/e_ad_conv.h>
#include <a_d/advance_ad_scan/e_micro.h>
#include <a_d/advance_ad_scan/e_prox.h>
#include <acc_gyro/e_lsm330.h>
#include <camera/fast_2_timer/e_po8030d.h>
#include <camera/fast_2_timer/e_poxxxx.h>
#include <codec/e_sound.h>
#include <ctype.h>
#include <hal.h>
#include <main.h>
#include <motor_led/advance_one_timer/e_agenda.h>
#include <motor_led/advance_one_timer/e_led.h>
#include <motor_led/advance_one_timer/e_motors.h>
#include <motor_led/e_epuck_ports.h>
#include <motor_led/e_init_port.h>
#include <stdio.h>
#include <string.h>
#include <uart/e_uart_char.h>
#include <utility/utility.h>
#ifdef CLIFF_SENSORS
#ifndef FLOOR_SENSORS
#define FLOOR_SENSORS
#endif
#endif
#ifdef FLOOR_SENSORS
#include <I2C/e_I2C_protocol.h>
#endif

#ifdef IR_RECEIVER
#include <motor_led/advance_one_timer/e_remote_control.h>
#define SPEED_IR 600
#endif
#include "DataEEPROM.h"

#include "memory.h"

extern char buffer[BUFFER_SIZE];
#define IMAGE_HEADER_SIZE 3 // mode, width, height
#define IMAGE_MAX_SIZE (BUFFER_SIZE - IMAGE_HEADER_SIZE)

typedef void (*command_handler_t)(BaseSequentialStream *uart_target);
void asercom_default_handler(BaseSequentialStream *uart_target) { return; }

void asercom_get_accelerometer_handler_ascii_A(BaseSequentialStream *uart_target) {
    snprintf(buffer, BUFFER_SIZE, "a,%d,%d,%d\r\n", e_get_acc_filtered(0, 1), e_get_acc_filtered(1, 1),
             e_get_acc_filtered(2, 1));
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
}

void asercom_get_battery_state_handler_ascii_b(BaseSequentialStream *uart_target) {
    snprintf(buffer, BUFFER_SIZE, "b,%d (%d%%)\r\n", getBatteryValueRaw(), getBatteryValuePercentage());
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
}

void asercom_set_body_led_handler_ascii_B(BaseSequentialStream *uart_target) {
    int LED_action = 0;
    sscanf(buffer, "B,%d\r", &LED_action);
    e_set_body_led(LED_action);
    char s[] = "b\r\n";
    chSequentialStreamWrite(uart_target, s, sizeof(s));
}

void asercom_read_selector_position_handler_ascii_C(BaseSequentialStream *uart_target) {
    int selector = SELECTOR0 + 2 * SELECTOR1 + 4 * SELECTOR2 + 8 * SELECTOR3;
    sprintf(buffer, "c,%d\r\n", selector);
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
}

void asercom_set_motor_speed_ascii_D(BaseSequentialStream *uart_target) {
    int speedl = 0, speedr = 0;
    sscanf(buffer, "D,%d,%d\r", &speedl, &speedr);
    e_set_speed_left(speedl);
    e_set_speed_right(speedr);
    chSequentialStreamWrite(uart_target, "d\r\n", 4);
}

// TODO: implement this, not implemented in the original code too.
void asercom_read_motor_speed_ascii_E(BaseSequentialStream *uart_target) {
    sprintf(buffer, "e,%d,%d\r\n", "unavailable", "unavailable");
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
}

void asercom_set_front_led_ascii_F(BaseSequentialStream *uart_target) {
    int LED_action = 0;
    sscanf(buffer, "F,%d\r", &LED_action);
    e_set_front_led(LED_action);
    chSequentialStreamWrite(uart_target, "f\r\n", 4);
}

// FIXME: Removed because caused an error and I am lazy.
void asercom_get_ir_remote_status_ascii_G(BaseSequentialStream *uart_target) {
    // sprintf(buffer, "g IR check : 0x%x, address : 0x%x, data : 0x%x\r\n", e_get_check(), e_get_address(),
    // e_get_data()); chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
}

void asercom_get_gyro_rates_ascii_g(BaseSequentialStream *uart_target) {
    sprintf(buffer, "g,%d,%d,%d\r\n", getXAxisGyro(), getYAxisGyro(), getZAxisGyro());
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
}

void asercom_get_help_message_ascii_H(BaseSequentialStream *uart_target) {
    chSequentialStreamWrite(uart_target, "\n", 2);
    chSequentialStreamWrite(uart_target, "\"A\"               Accelerometer\r\n", 34);
    chSequentialStreamWrite(uart_target, "\"B,#\"             Body led 0=off 1=on 2=inverse\r\n", 50);
    chSequentialStreamWrite(uart_target, "\"b\"               Battery value\r\n", 34);
    chSequentialStreamWrite(uart_target, "\"b\"               Battery state (1=ok, 0=low)\r\n", 48);
    chSequentialStreamWrite(uart_target, "\"C\"               Selector position\r\n", 38);
    chSequentialStreamWrite(uart_target, "\"D,#,#\"           Set motor speed left,right\r\n", 47);
    chSequentialStreamWrite(uart_target, "\"E\"               Get motor speed left,right\r\n", 47);
    chSequentialStreamWrite(uart_target, "\"F,#\"             Front led 0=off 1=on 2=inverse\r\n", 51);
    chSequentialStreamWrite(uart_target, "\"G\"               IR receiver\r\n", 32);
    chSequentialStreamWrite(uart_target, "\"g\"               Gyro\r\n", 25);
    chSequentialStreamWrite(uart_target, "\"H\"               Help\r\n", 25);
    chSequentialStreamWrite(uart_target, "\"I\"               Get camera parameter\r\n", 41);
    chSequentialStreamWrite(uart_target,
                            "\"J,#,#,#,#,#,#\"   Set camera parameter mode,width,heigth,zoom(1,4 or 8),x1,y1\r\n", 80);
    chSequentialStreamWrite(uart_target, "\"K\"               Calibrate proximity sensors\r\n", 48);
    chSequentialStreamWrite(uart_target, "\"L,#,#\"           Led number,0=off 1=on 2=inverse\r\n", 52);
    chSequentialStreamWrite(uart_target, "\"M\"               Floor sensors\r\n", 34);
    chSequentialStreamWrite(uart_target, "\"N\"               Proximity\r\n", 30);
    chSequentialStreamWrite(uart_target, "\"O\"               Light sensors\r\n", 34);
    chSequentialStreamWrite(uart_target, "\"P,#,#\"           Set motor position left,right\r\n", 50);
    chSequentialStreamWrite(uart_target, "\"Q\"               Get motor position left,right\r\n", 50);
    chSequentialStreamWrite(uart_target, "\"R\"               Reset e-puck\r\n", 33);
    chSequentialStreamWrite(uart_target, "\"S\"               Stop e-puck and turn off leds\r\n", 50);
    chSequentialStreamWrite(uart_target, "\"T,#\"             Play sound 1-5 else stop sound\r\n", 51);
    chSequentialStreamWrite(uart_target, "\"t\"               Temperature\r\n", 32);
    chSequentialStreamWrite(uart_target, "\"U\"               Get microphone amplitude\r\n", 45);
    chSequentialStreamWrite(uart_target, "\"V\"               Version of SerCom\r\n", 38);
    chSequentialStreamWrite(uart_target, "\"W\"               Write I2C (mod,reg,val)\r\n", 44);
    chSequentialStreamWrite(uart_target, "\"Y\"               Read I2C val=(mod,reg)\r\n", 43);
}

typedef struct {
    int mode, width, heigth, zoom, size, x1, y1;
} asercom_camera_parameter_t;
asercom_camera_parameter_t asercom_camera_parameters = {
    .mode = RGB_565_MODE, .width = 40, .heigth = 40, .zoom = 8, .size = 40 * 40 * 2};

// TODO: add camera information
void asercom_get_camera_info_ascii_I(BaseSequentialStream *uart_target) {
    sprintf(buffer, "i,%d,%d,%d,%d,%d\r\n", &asercom_camera_parameters.mode, &asercom_camera_parameters.width,
            &asercom_camera_parameters.heigth, &asercom_camera_parameters.zoom, &asercom_camera_parameters.size);
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
}

void asercom_set_camera_parameter_ascii_J(BaseSequentialStream *uart_target) {
    asercom_camera_parameters.x1 = -1;
    asercom_camera_parameters.y1 = -1;
    sscanf(buffer, "J,%d,%d,%d,%d,%d,%d\r", &asercom_camera_parameters.mode, &asercom_camera_parameters.width,
           &asercom_camera_parameters.heigth, &asercom_camera_parameters.zoom, &asercom_camera_parameters.x1,
           &asercom_camera_parameters.y1);
    if (asercom_camera_parameters.mode == GREY_SCALE_MODE)
        asercom_camera_parameters.size = asercom_camera_parameters.width * asercom_camera_parameters.heigth;
    else
        asercom_camera_parameters.size = asercom_camera_parameters.width * asercom_camera_parameters.heigth * 2;
    if (asercom_camera_parameters.size >
        IMAGE_MAX_SIZE) { // if desired settings too demanding set to a reasonable default
        asercom_camera_parameters.mode = RGB_565_MODE;
        asercom_camera_parameters.width = 40;  // DEFAULT_WIDTH;
        asercom_camera_parameters.heigth = 40; // DEFAULT_HEIGHT;
        asercom_camera_parameters.size = asercom_camera_parameters.width * asercom_camera_parameters.heigth * 2;
    }
    e_poxxxx_init_cam();
    if (asercom_camera_parameters.y1 == -1) { // user did not specify: take default
        asercom_camera_parameters.y1 =
            (ARRAY_WIDTH - asercom_camera_parameters.width * asercom_camera_parameters.zoom) / 2;
    }
    if (asercom_camera_parameters.y1 == -1) { // user did not specify: take default
        asercom_camera_parameters.y1 =
            (ARRAY_HEIGHT - asercom_camera_parameters.heigth * asercom_camera_parameters.zoom) / 2;
    }
    e_poxxxx_config_cam(asercom_camera_parameters.y1, asercom_camera_parameters.y1,
                        asercom_camera_parameters.width * asercom_camera_parameters.zoom,
                        asercom_camera_parameters.heigth * asercom_camera_parameters.zoom,
                        asercom_camera_parameters.zoom, asercom_camera_parameters.zoom, asercom_camera_parameters.mode);
    e_poxxxx_write_cam_registers();
    chSequentialStreamWrite(uart_target, "b\r\n", 4);
}

void asercom_calibrate_proximity_sensors_ascii_K(BaseSequentialStream *uart_target) {
    chSequentialStreamWrite(uart_target, "k, Starting calibration - Remove any object in sensors range\r\n", 63);
    int long t;
    e_set_led(8, 1);
    for (t = 0; t < 1000000; ++t)
        ;
    chThdSleepMilliseconds(400);
    e_led_clear();
    for (t = 0; t < 10000; ++t)
        ;
    chThdSleepMilliseconds(100);
    e_calibrate_ir();
    chSequentialStreamWrite(uart_target, "k, Calibration finished\r\n", 26);
}

void asercom_set_led_ascii_L(BaseSequentialStream *uart_target) {
    int LED_nbr = 0, LED_action = 0;
    sscanf(buffer, "L,%d,%d\r", &LED_nbr, &LED_action);
    e_set_led(LED_nbr, LED_action);
    chSequentialStreamWrite(uart_target, "l\r\n", 4);
}

void asercom_get_floor_sensors_ascii_M(BaseSequentialStream *uart_target) {
    sprintf(buffer, "m,%d,%d,%d,%d,%d\r\n", get_ground_prox(0), get_ground_prox(1), get_ground_prox(2),
            get_ground_prox(3), get_ground_prox(4));
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
}

void asercom_get_proximity_sensors_ascii_N(BaseSequentialStream *uart_target) {
    sprintf(buffer, "n,%d,%d,%d,%d,%d,%d,%d,%d\r\n", e_get_calibrated_prox(0), e_get_calibrated_prox(1),
            e_get_calibrated_prox(2), e_get_calibrated_prox(3), e_get_calibrated_prox(4), e_get_calibrated_prox(5),
            e_get_calibrated_prox(6), e_get_calibrated_prox(7));
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
}

void asercom_get_ambient_light_ascii_I(BaseSequentialStream *uart_target) {
    sprintf(buffer, "o,%d,%d,%d,%d,%d,%d,%d,%d\r\n", e_get_ambient_light(0), e_get_ambient_light(1),
            e_get_ambient_light(2), e_get_ambient_light(3), e_get_ambient_light(4), e_get_ambient_light(5),
            e_get_ambient_light(6), e_get_ambient_light(7));
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
}

void asercom_set_motor_steps_ascii_P(BaseSequentialStream *uart_target) {
    int positionl = 0, positionr = 0;
    sscanf(buffer, "P,%d,%d\r", &positionl, &positionr);
    e_set_steps_left(positionl);
    e_set_steps_right(positionr);
    chSequentialStreamWrite(uart_target, "p\r\n", 4);
}

void asercom_get_motor_steps_ascii_Q(BaseSequentialStream *uart_target) {
    sprintf(buffer, "q,%d,%d\r\n", e_get_steps_left(), e_get_steps_right());
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
}

void asercom_reset_ascii_R(BaseSequentialStream *uart_target) {
    // TODO: no implementation yet.
    return;
}

void asercom_stop_ascii_S(BaseSequentialStream *uart_target) {
    e_set_speed_left(0);
    e_set_speed_right(0);
    e_set_led(8, 0);
    chSequentialStreamWrite(uart_target, "s\r\n", 4);
}

void asercom_stop_with_music_ascii_T(BaseSequentialStream *uart_target) {
    // TODO: implement this, I don't care about music right now.
    return;
}

void asercom_get_temperature_ascii_t(BaseSequentialStream *uart_target) {
    sprintf(buffer, "t,%d\r\n", getTemperature());
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
}

void asercom_get_microphone_volume_ascii_U(BaseSequentialStream *uart_target) {
    sprintf(buffer, "u,%d,%d,%d\r\n", e_get_micro_volume(0), e_get_micro_volume(1), e_get_micro_volume(2));
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
}

void asercom_get_version_ascii_V(BaseSequentialStream *uart_target) {
    // TODO: update this once the project is finished.
    chSequentialStreamWrite(uart_target, "v,Version ???", 14);
}

// Binary sequences
void asercom_get_all_sensors_binary_0xF8(BaseSequentialStream *uart_target) {
    // Accelerometers
    int i = 0;

    int accx = e_get_acc(0);
    int accy = e_get_acc(1);
    int accz = e_get_acc(2);

    buffer[i++] = accx & 0xff;
    buffer[i++] = accx >> 8;
    buffer[i++] = accy & 0xff;
    buffer[i++] = accy >> 8;
    buffer[i++] = accz & 0xff;
    buffer[i++] = accz >> 8;

    TypeAccSpheric accelero = e_read_acc_spheric();

    char *ptr = (char *)&accelero.acceleration;
    buffer[i++] = (*ptr);
    ptr++;
    buffer[i++] = (*ptr);
    ptr++;
    buffer[i++] = (*ptr);
    ptr++;
    buffer[i++] = (*ptr);

    ptr = (char *)&accelero.orientation;
    buffer[i++] = (*ptr);
    ptr++;
    buffer[i++] = (*ptr);
    ptr++;
    buffer[i++] = (*ptr);
    ptr++;
    buffer[i++] = (*ptr);

    ptr = (char *)&accelero.inclination;
    buffer[i++] = (*ptr);
    ptr++;
    buffer[i++] = (*ptr);
    ptr++;
    buffer[i++] = (*ptr);
    ptr++;
    buffer[i++] = (*ptr);

    // Gyroscope
    int gyrox, gyroy, gyroz;
    getAllAxesGyro(&gyrox, &gyroy, &gyroz);
    buffer[i++] = gyrox & 0xFF;
    buffer[i++] = gyrox >> 8;
    buffer[i++] = gyroy & 0xFF;
    buffer[i++] = gyroy >> 8;
    buffer[i++] = gyroz & 0xFF;
    buffer[i++] = gyroz >> 8;

    // Magnetometer
    float tempf;
    int tempi;
    for (int j = 0; j < 3; j++) {
        tempf = get_magnetic_field(j);
        tempi = *((uint32_t *)&tempf);
        buffer[i++] = tempi & 0xff;
        buffer[i++] = tempi >> 8;
        buffer[i++] = tempi >> 16;
        buffer[i++] = tempi >> 24;
    }

    // Temperature
    buffer[i++] = getTemperature();

    // Proximity
}

command_handler_t command_table[256] = {
    // set all the entries to a default handler.
    [0 ... 255] = asercom_default_handler,
    ['A'] = asercom_get_accelerometer_handler_ascii_A,
    ['b'] = asercom_get_battery_state_handler_ascii_b,
    ['B'] = asercom_set_body_led_handler_ascii_B,
    ['C'] = asercom_read_selector_position_handler_ascii_C,
    ['D'] = asercom_set_motor_speed_ascii_D,
    ['E'] = asercom_read_motor_speed_ascii_E,
    ['F'] = asercom_set_front_led_ascii_F,
    ['G'] = asercom_get_ir_remote_status_ascii_G,
    ['g'] = asercom_get_gyro_rates_ascii_g,
    ['H'] = asercom_get_help_message_ascii_H,
    ['I'] = asercom_get_camera_info_ascii_I,
    ['J'] = asercom_set_camera_parameter_ascii_J,
    ['K'] = asercom_calibrate_proximity_sensors_ascii_K,
    ['L'] = asercom_set_led_ascii_L,
    ['M'] = asercom_get_floor_sensors_ascii_M,
    ['N'] = asercom_get_proximity_sensors_ascii_N,
    ['I'] = asercom_get_ambient_light_ascii_I,
    ['P'] = asercom_set_motor_steps_ascii_P,
    ['Q'] = asercom_get_motor_steps_ascii_Q,
    ['R'] = asercom_reset_ascii_R,
    ['S'] = asercom_stop_ascii_S,
    ['T'] = asercom_stop_with_music_ascii_T,
    ['t'] = asercom_get_temperature_ascii_t,
    ['U'] = asercom_get_microphone_volume_ascii_U,
    ['V'] = asercom_get_version_ascii_V,
};

void run_asercom3(BaseSequentialStream *uart_target) {
    char c = 0;
    while (1) {
        while (chSequentialStreamRead(uart_target, &c, 1)) {
            command_table[c](uart_target);
        }
    }
}