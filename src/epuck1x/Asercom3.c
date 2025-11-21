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

/// @brief Buffer where data is written to before being flushed to the SequentialStream
extern char buffer[BUFFER_SIZE];
#define IMAGE_HEADER_SIZE 3 // mode, width, height
#define IMAGE_MAX_SIZE (BUFFER_SIZE - IMAGE_HEADER_SIZE)

/**
 * @brief Function definition for the asercom3 command handlers.
 * @details The typedef serves as a template for creatin new functions that recieve commands. Once you create a new
 * command, you must add the function to the dispatch table `command_table` at the right index. Make sure to name it
 * accordingly..
 *
 * @param buffer_offset Is an integer telling where the counting of the buffer offset should continue, so as to not
 * overwrite previous data. In ascii commands, it can be 0 since the buffer always gets flushed for each command.
 * @param uart_target Is a pointer to a BaseSequentialStream. It can be any, but will usually be `&SD2` for
 * communication over wifi or `&SDU1` over usb on `/dev/ttyACM2`.
 *
 * @return The buffer offset for the next function call.
 *
 */
typedef int (*command_handler_t)(int buffer_offset, BaseSequentialStream *uart_target);
int asercom_default_handler(int buffer_offset, BaseSequentialStream *uart_target) { return 0; }

int asercom_get_accelerometer_handler_ascii_A(int buffer_offset, BaseSequentialStream *uart_target) {
    snprintf(buffer, BUFFER_SIZE, "a,%d,%d,%d\r\n", e_get_acc_filtered(0, 1), e_get_acc_filtered(1, 1),
             e_get_acc_filtered(2, 1));
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
    return 0;
}

int asercom_get_battery_state_handler_ascii_b(int buffer_offset, BaseSequentialStream *uart_target) {
    snprintf(buffer, BUFFER_SIZE, "b,%d (%d%%)\r\n", getBatteryValueRaw(), getBatteryValuePercentage());
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
    return 0;
}

int asercom_set_body_led_handler_ascii_B(int buffer_offset, BaseSequentialStream *uart_target) {
    int LED_action = 0;
    sscanf(buffer, "B,%d\r", &LED_action);
    e_set_body_led(LED_action);
    char s[] = "b\r\n";
    chSequentialStreamWrite(uart_target, s, sizeof(s));
    return 0;
}

int asercom_read_selector_position_handler_ascii_C(int buffer_offset, BaseSequentialStream *uart_target) {
    int selector = SELECTOR0 + 2 * SELECTOR1 + 4 * SELECTOR2 + 8 * SELECTOR3;
    sprintf(buffer, "c,%d\r\n", selector);
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
    return 0;
}

int asercom_set_motor_speed_ascii_D(int buffer_offset, BaseSequentialStream *uart_target) {
    int speedl = 0, speedr = 0;
    sscanf(buffer, "D,%d,%d\r", &speedl, &speedr);
    e_set_speed_left(speedl);
    e_set_speed_right(speedr);
    chSequentialStreamWrite(uart_target, "d\r\n", 4);
    return 0;
}

// TODO: implement this, not implemented in the original code too.
int asercom_read_motor_speed_ascii_E(int buffer_offset, BaseSequentialStream *uart_target) {
    sprintf(buffer, "e,%d,%d\r\n", "unavailable", "unavailable");
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
    return 0;
}

int asercom_set_front_led_ascii_F(int buffer_offset, BaseSequentialStream *uart_target) {
    int LED_action = 0;
    sscanf(buffer, "F,%d\r", &LED_action);
    e_set_front_led(LED_action);
    chSequentialStreamWrite(uart_target, "f\r\n", 4);
    return 0;
}

// FIXME: Removed because caused an error and I am lazy.
int asercom_get_ir_remote_status_ascii_G(int buffer_offset, BaseSequentialStream *uart_target) {
    // sprintf(buffer, "g IR check : 0x%x, address : 0x%x, data : 0x%x\r\n", e_get_check(), e_get_address(),
    // e_get_data()); chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
    return 0;
}

int asercom_get_gyro_rates_ascii_g(int buffer_offset, BaseSequentialStream *uart_target) {
    sprintf(buffer, "g,%d,%d,%d\r\n", getXAxisGyro(), getYAxisGyro(), getZAxisGyro());
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
    return 0;
}

int asercom_get_help_message_ascii_H(int buffer_offset, BaseSequentialStream *uart_target) {
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
    return 0;
}

typedef struct {
    int mode, width, heigth, zoom, size, x1, y1;
} asercom_camera_parameter_t;
asercom_camera_parameter_t asercom_camera_parameters = {
    .mode = RGB_565_MODE, .width = 40, .heigth = 40, .zoom = 8, .size = 40 * 40 * 2};

// TODO: add camera information
int asercom_get_camera_info_ascii_I(int buffer_offset, BaseSequentialStream *uart_target) {
    sprintf(buffer, "i,%d,%d,%d,%d,%d\r\n", &asercom_camera_parameters.mode, &asercom_camera_parameters.width,
            &asercom_camera_parameters.heigth, &asercom_camera_parameters.zoom, &asercom_camera_parameters.size);
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
    return 0;
}

int asercom_set_camera_parameter_ascii_J(int buffer_offset, BaseSequentialStream *uart_target) {
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
    return 0;
}

int asercom_calibrate_proximity_sensors_ascii_K(int buffer_offset, BaseSequentialStream *uart_target) {
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
    return 0;
}

int asercom_set_led_ascii_L(int buffer_offset, BaseSequentialStream *uart_target) {
    int LED_nbr = 0, LED_action = 0;
    sscanf(buffer, "L,%d,%d\r", &LED_nbr, &LED_action);
    e_set_led(LED_nbr, LED_action);
    chSequentialStreamWrite(uart_target, "l\r\n", 4);
    return 0;
}

int asercom_get_floor_sensors_ascii_M(int buffer_offset, BaseSequentialStream *uart_target) {
    sprintf(buffer, "m,%d,%d,%d,%d,%d\r\n", get_ground_prox(0), get_ground_prox(1), get_ground_prox(2),
            get_ground_prox(3), get_ground_prox(4));
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
    return 0;
}

int asercom_get_proximity_sensors_ascii_N(int buffer_offset, BaseSequentialStream *uart_target) {
    sprintf(buffer, "n,%d,%d,%d,%d,%d,%d,%d,%d\r\n", e_get_calibrated_prox(0), e_get_calibrated_prox(1),
            e_get_calibrated_prox(2), e_get_calibrated_prox(3), e_get_calibrated_prox(4), e_get_calibrated_prox(5),
            e_get_calibrated_prox(6), e_get_calibrated_prox(7));
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
    return 0;
}

int asercom_get_ambient_light_ascii_I(int buffer_offset, BaseSequentialStream *uart_target) {
    sprintf(buffer, "o,%d,%d,%d,%d,%d,%d,%d,%d\r\n", e_get_ambient_light(0), e_get_ambient_light(1),
            e_get_ambient_light(2), e_get_ambient_light(3), e_get_ambient_light(4), e_get_ambient_light(5),
            e_get_ambient_light(6), e_get_ambient_light(7));
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
    return 0;
}

int asercom_set_motor_steps_ascii_P(int buffer_offset, BaseSequentialStream *uart_target) {
    int positionl = 0, positionr = 0;
    sscanf(buffer, "P,%d,%d\r", &positionl, &positionr);
    e_set_steps_left(positionl);
    e_set_steps_right(positionr);
    chSequentialStreamWrite(uart_target, "p\r\n", 4);
    return 0;
}

int asercom_get_motor_steps_ascii_Q(int buffer_offset, BaseSequentialStream *uart_target) {
    sprintf(buffer, "q,%d,%d\r\n", e_get_steps_left(), e_get_steps_right());
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
    return 0;
}

int asercom_reset_ascii_R(int buffer_offset, BaseSequentialStream *uart_target) {
    // TODO: no implementation yet.
    return 0;
}

int asercom_stop_ascii_S(int buffer_offset, BaseSequentialStream *uart_target) {
    e_set_speed_left(0);
    e_set_speed_right(0);
    e_set_led(8, 0);
    chSequentialStreamWrite(uart_target, "s\r\n", 4);
    return 0;
}

int asercom_stop_with_music_ascii_T(int buffer_offset, BaseSequentialStream *uart_target) {
    // TODO: implement this, I don't care about music right now.
    return 0;
}

int asercom_get_temperature_ascii_t(int buffer_offset, BaseSequentialStream *uart_target) {
    sprintf(buffer, "t,%d\r\n", getTemperature());
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
    return 0;
}

int asercom_get_microphone_volume_ascii_U(int buffer_offset, BaseSequentialStream *uart_target) {
    sprintf(buffer, "u,%d,%d,%d\r\n", e_get_micro_volume(0), e_get_micro_volume(1), e_get_micro_volume(2));
    chSequentialStreamWrite(uart_target, buffer, strlen(buffer));
    return 0;
}

int asercom_get_version_ascii_V(int buffer_offset, BaseSequentialStream *uart_target) {
    // TODO: update this once the project is finished.
    chSequentialStreamWrite(uart_target, "v,Version ???", 14);
    return 0;
}

// Binary sequences
int asercom_get_all_sensors_binary_0xF8(int buffer_offset, BaseSequentialStream *uart_target) {
    // Accelerometers
    int i = buffer_offset;

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
    int n;
    for (int j = 0; j < 8; j++) {
        n = e_get_calibrated_prox(j); // or ? n=e_get_prox(j);
        buffer[i++] = n & 0xff;
        buffer[i++] = n >> 8;
    }

    // Ambient light
    for (int j = 0; j < 8; j++) {
        n = e_get_ambient_light(j);
        buffer[i++] = n & 0xff;
        buffer[i++] = n >> 8;
    }

    // Time-of-flight
    n = VL53L0X_get_dist_mm();
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;

    // Microphone volume
    n = e_get_micro_volume(0);
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;
    n = e_get_micro_volume(1);
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;
    n = e_get_micro_volume(2);
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;
    n = e_get_micro_volume(3);
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;

    // Motor steps
    n = e_get_steps_left();
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;
    n = e_get_steps_right();
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;

    // Battery
    unsigned int battValue = getBatteryValueRaw();
    buffer[i++] = battValue & 0xFF;
    buffer[i++] = battValue >> 8;

    // Micro SD state
    if (sdio_is_present()) {
        n = !sdio_connect();
        buffer[i++] = n & 0xff;
        sdio_disconnect();
    } else {
        buffer[i++] = 0;
    }

    // IR Remote
    // FIXME: currently disabled to remove an error
    buffer[i++] = 0; // e_get_check();
    buffer[i++] = 0; // e_get_address();
    buffer[i++] = 0; // e_get_data();

    // Selector
    int selector = SELECTOR0 + 2 * SELECTOR1 + 4 * SELECTOR2 + 8 * SELECTOR3;
    buffer[i++] = selector;

    // Ground sensors proximity
    for (int j = 0; j < 3; j++) {
        n = get_ground_prox(j);
        buffer[i++] = n & 0xff;
        buffer[i++] = n >> 8;
    }

    // Ground sensors ambient light
    for (int j = 0; j < 3; j++) {
        n = get_ground_ambient_light(j);
        buffer[i++] = n & 0xff;
        buffer[i++] = n >> 8;
    }

    // Additional empty byte for future use.
    buffer[i++] = 0;
    return i;
}

/// @brief Buffer to store data from the SequentialStream.
uint8_t rx_buff[19] = {0};

int asercom_set_all_actuators_binary_0xF7(int buffer_offset, BaseSequentialStream *uart_target) {
    chSequentialStreamRead(uart_target, (uint8_t *)rx_buff, 19);

    if (serial_get_last_errors() != 0) {
        serial_clear_last_errors();
        return buffer_offset; // skipping packet
    }

    // Handle behaviors and others commands.
    if (rx_buff[0] & 0x01) { // Calibrate proximity.
        calibrate_ir();
    }
    if (rx_buff[0] & 0x02) { // Enable obastacle avoidance.
        enable_obstacle_avoidance();
    } else { // Disable obstacle avoidance
        disable_obstacle_avoidance();
    }

    // Set motor speed or motor position.
    int speedl = (unsigned char)rx_buff[1] + ((unsigned int)rx_buff[2] << 8);
    int speedr = (unsigned char)rx_buff[3] + ((unsigned int)rx_buff[4] << 8);

    if (rx_buff[0] & 0x04) { // Set steps.
        e_set_steps_left(speedl);
        e_set_steps_right(speedr);
    } else { // Set speed.
        e_set_speed_left(speedl);
        e_set_speed_right(speedr);
    }

    // Set LEDs.
    if (rx_buff[5] & 0x01) {
        set_led(LED1, 1);
    } else {
        set_led(LED1, 0);
    }
    if (rx_buff[5] & 0x02) {
        set_led(LED3, 1);
    } else {
        set_led(LED3, 0);
    }
    if (rx_buff[5] & 0x04) {
        set_led(LED5, 1);
    } else {
        set_led(LED5, 0);
    }
    if (rx_buff[5] & 0x08) {
        set_led(LED7, 1);
    } else {
        set_led(LED7, 0);
    }
    if (rx_buff[5] & 0x10) {
        set_body_led(1);
    } else {
        set_body_led(0);
    }
    if (rx_buff[5] & 0x20) {
        set_front_led(1);
    } else {
        set_front_led(0);
    }

    // RGBs setting.
    set_rgb_led(0, rx_buff[6], rx_buff[7], rx_buff[8]);
    set_rgb_led(1, rx_buff[9], rx_buff[10], rx_buff[11]);
    set_rgb_led(2, rx_buff[12], rx_buff[13], rx_buff[14]);
    set_rgb_led(3, rx_buff[15], rx_buff[16], rx_buff[17]);

    // Play sound.
    if (rx_buff[18] & 0x01) {
        playMelody(MARIO, ML_FORCE_CHANGE, NULL); // e_play_sound(0, 2112);
    }
    if (rx_buff[18] & 0x02) {
        playMelody(UNDERWORLD, ML_FORCE_CHANGE, NULL); // e_play_sound(2116, 1760);
    }
    if (rx_buff[18] & 0x04) {
        playMelody(STARWARS, ML_FORCE_CHANGE, NULL); // e_play_sound(3878, 3412);
    }
    if (rx_buff[18] & 0x08) {
        e_play_sound(7294, 3732);
    }
    if (rx_buff[18] & 0x10) {
        e_play_sound(11028, 8016);
    }
    if (rx_buff[18] & 0x20) {
        e_close_sound();
        stopCurrentMelody();
    }

    return buffer_offset;
}

int asercom_set_rgb_led_binary_0xF6(int buffer_offset, BaseSequentialStream *uart_target) {
    char rgb_value[12];

    for (int j = 0; j < 12; j++) {
        // TODO: look into how this works, because I don't understand the == 0.
        while (chSequentialStreamRead(uart_target, &rgb_value[j], 1) == 0)
            ;
    }

    set_rgb_led(0, rgb_value[0], rgb_value[1], rgb_value[2]);
    set_rgb_led(1, rgb_value[3], rgb_value[4], rgb_value[5]);
    set_rgb_led(2, rgb_value[6], rgb_value[7], rgb_value[8]);
    set_rgb_led(3, rgb_value[9], rgb_value[10], rgb_value[11]);
    return buffer_offset;
}

int asercom_get_button_state_binary_0xF5(int buffer_offset, BaseSequentialStream *uart_target) {
    buffer[buffer_offset++] = button_get_state();
    return buffer_offset;
}

int asercom_get_microphones_binary_0xF4(int buffer_offset, BaseSequentialStream *uart_target) {
    int i = buffer_offset;
    int n = e_get_micro_volume(0);
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;

    n = e_get_micro_volume(1);
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;

    n = e_get_micro_volume(2);
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;

    n = e_get_micro_volume(3);
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;
    return i;
}

int asercom_get_time_of_flight_binary_0xF3(int buffer_offset, BaseSequentialStream *uart_target) {
    int i = buffer_offset;
    int n = VL53L0X_get_dist_mm();
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;
    return i;
}

int asercom_get_sd_card_state_binary_0xF2(int buffer_offset, BaseSequentialStream *uart_target) {
    int i = buffer_offset;
    if (sdio_is_present()) {
        int n = !sdio_connect();
        buffer[i++] = n & 0xff;
        sdio_disconnect();
    } else {
        buffer[i++] = 0;
    }

    return i;
}

int asercom_enable_disable_magnetometer_binary_0xF1(int buffer_offset, BaseSequentialStream *uart_target) {
    chSequentialStreamRead(uart_target, rx_buff, 1);

    // In case of errors, skip the packet.
    if (serial_get_last_errors() != 0) {
        serial_clear_last_errors();
        return buffer_offset;
    }

    if (rx_buff[0] == 1) {
        mpu9250_magnetometer_setup();
    } else {
        // Magnetometer stop not yet implemented.
    }

    buffer[buffer_offset++] = 0; // success
    return buffer_offset;
}

// Set proximity state (0=disable, 1=enable fast sampling, 2=enable slow sampling)
int asercom_set_proximity_state_binary_0xF0(int buffer_offset, BaseSequentialStream *uart_target) {
    chSequentialStreamRead(uart_target, rx_buff, 1);

    if (serial_get_last_errors() != 0) {
        // sprintf(buffer, "skip packet\r\n");
        // uart2_send_text(buffer);
        serial_clear_last_errors();
        return buffer_offset;
    }

    proximity_stop();
    if (rx_buff[0] == 1) {
        proximity_start(FAST_UPDATE);
    } else if (rx_buff[0] == 2) {
        proximity_start(SLOW_UPDATE);
    }

    buffer[buffer_offset++] = 0; // success
    return buffer_offset;
}

int asercom_enable_disable_time_of_flight_binary_0xEF(int buffer_offset, BaseSequentialStream *uart_target) {
    chSequentialStreamRead(uart_target, rx_buff, 1);

    // In case of errors, skip the packet.
    if (serial_get_last_errors() != 0) {
        // sprintf(buffer, "skip packet\r\n");
        // uart2_send_text(buffer);
        serial_clear_last_errors();
        return buffer_offset;
    }

    if (rx_buff[0] == 0) {
        VL53L0X_stop();
    } else {
        VL53L0X_start();
    }

    buffer[buffer_offset++] = 0; // success
    return buffer_offset;
}

int asercom_get_all_sensors_compact_binary_0xEE(int buffer_offset, BaseSequentialStream *uart_target) {
    int i = buffer_offset, n;

    // Read accelerometer.
    int16_t pitch = get_pitch();
    int16_t roll = get_roll();
    int16_t yaw = get_yaw();
    uint8_t acc_accuracy = get_acc_accuracy();
    uint8_t gyro_accuracy = get_gyro_accuracy();
    uint8_t mag_accuracy = get_mag_accuracy();
    buffer[i++] = pitch & 0xff;
    buffer[i++] = pitch >> 8;
    buffer[i++] = roll & 0xff;
    buffer[i++] = roll >> 8;
    buffer[i++] = yaw & 0xff;
    buffer[i++] = yaw >> 8;
    buffer[i++] = acc_accuracy;
    buffer[i++] = gyro_accuracy;
    buffer[i++] = mag_accuracy;

    // Read temperature.
    buffer[i++] = getTemperature();

    // Read proximities.
    for (int j = 0; j < 8; j++) {
        n = e_get_calibrated_prox(j); // or ? n=e_get_prox(j);
        buffer[i++] = n & 0xff;
        buffer[i++] = n >> 8;
    }

    // Read ToF.
    n = VL53L0X_get_dist_mm();
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;

    // Read microphones.
    n = e_get_micro_volume(0);
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;
    n = e_get_micro_volume(1);
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;
    n = e_get_micro_volume(2);
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;
    n = e_get_micro_volume(3);
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;

    // Read encoders.
    n = e_get_steps_left();
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;
    n = e_get_steps_right();
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;

    // Read battery.
    unsigned int battValue = getBatteryValueRaw();
    buffer[i++] = battValue & 0xFF;
    buffer[i++] = battValue >> 8;

    // Read micro sd state.
    if (sdio_is_present()) {
        n = !sdio_connect();
        buffer[i++] = n & 0xff;
        sdio_disconnect();
    } else {
        buffer[i++] = 0;
    }

    // Read tv remote.
    // buffer[i++] = e_get_data();
    // FIXME: disabled because lazy

    // Read selector.
    char selector = SELECTOR0 + 2 * SELECTOR1 + 4 * SELECTOR2 + 8 * SELECTOR3;
    buffer[i++] = selector;

    // Read ground sensor proximity.
    for (int j = 0; j < 3; j++) {
        n = get_ground_prox(j);
        buffer[i++] = n & 0xff;
        buffer[i++] = n >> 8;
    }

    // Button state => ESP32
    buffer[i++] = button_get_state();

    // Additional empty byte for future use.
    buffer[i++] = 0;
    return i;
}

int asercom_get_acceleration_raw_binary_0x9F(int buffer_offset, BaseSequentialStream *uart_target) {
    int i = buffer_offset;
    int accx = e_get_acc_filtered(0, 1);
    int accy = e_get_acc_filtered(1, 1);
    int accz = e_get_acc_filtered(2, 1);
    buffer[i++] = accx & 0xff;
    buffer[i++] = accx >> 8;
    buffer[i++] = accy & 0xff;
    buffer[i++] = accy >> 8;
    buffer[i++] = accz & 0xff;
    buffer[i++] = accz >> 8;
    return i;
}

int asercom_get_acceleration_binary_0xA1(int buffer_offset, BaseSequentialStream *uart_target) {
    TypeAccSpheric accelero = e_read_acc_spheric();
    char *ptr = (char *)&accelero.acceleration;
    int i = buffer_offset;
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
    return i;
}

int asercom_get_battery_state_binary_0x9E(int buffer_offset, BaseSequentialStream *uart_target) {
    unsigned int battValue = getBatteryValueRaw();
    buffer[buffer_offset++] = battValue & 0xFF;
    buffer[buffer_offset++] = battValue >> 8;
    return buffer_offset;
}

int asercom_set_motor_speed_binary_0xBC(int buffer_offset, BaseSequentialStream *uart_target) {
    char c1, c2;

    while (chSequentialStreamRead(uart_target, &c1, 1) == 0)
        ;
    while (chSequentialStreamRead(uart_target, &c2, 1) == 0)
        ;

    int speedl = (unsigned char)c1 + ((unsigned int)c2 << 8);

    while (chSequentialStreamRead(uart_target, &c1, 1) == 0)
        ;
    while (chSequentialStreamRead(uart_target, &c2, 1) == 0)
        ;

    int speedr = (unsigned char)c1 + ((unsigned int)c2 << 8);

    e_set_speed_left(speedl);
    e_set_speed_right(speedr);

    return buffer_offset;
}

// TODO: There is currently no smart way to do this without just taking the previous value.
int asercom_get_motor_speed_binary_0xBB(int buffer_offset, BaseSequentialStream *uart_target) {
    int i = buffer_offset;

    buffer[i++] = 0;
    buffer[i++] = 0;
    buffer[i++] = 0;
    buffer[i++] = 0;
    return i;
}

int asercom_get_gyroscope_binary_0x99(int buffer_offset, BaseSequentialStream *uart_target) {
    int i = buffer_offset, gyrox, gyroy, gyroz;
    getAllAxesGyro(&gyrox, &gyroy, &gyroz);
    buffer[i++] = gyrox & 0xFF;
    buffer[i++] = gyrox >> 8;
    buffer[i++] = gyroy & 0xFF;
    buffer[i++] = gyroy >> 8;
    buffer[i++] = gyroz & 0xFF;
    buffer[i++] = gyroz >> 8;

    return i;
}

int asercom_get_camera_image_binary_0xB7(int buffer_offset, BaseSequentialStream *uart_target) {
    int i = buffer_offset;
    e_poxxxx_launch_capture(&buffer[i + 3]);
    buffer[i++] = (char)asercom_camera_parameters.mode & 0xff; // send image parameter
    buffer[i++] = (char)asercom_camera_parameters.width & 0xff;
    buffer[i++] = (char)asercom_camera_parameters.heigth & 0xff;
    e_poxxxx_wait_img_ready();
    memcpy(&buffer[i], cam_get_last_image_ptr(), asercom_camera_parameters.size);
    i += asercom_camera_parameters.size;
    i++; // always +1 since the buffer offset should point to the next free byte.
    return i;
}

int asercom_set_led_binary_0xB4(int buffer_offset, BaseSequentialStream *uart_target) {
    char c1, c2;
    while (chSequentialStreamRead(uart_target, &c1, 1) == 0)
        ;
    while (chSequentialStreamRead(uart_target, &c2, 1) == 0)
        ;

    switch (c1) {
    case 8:
        e_set_body_led(c2);
        break;
    case 9:
        e_set_front_led(c2);
        break;
    default:
        e_set_led(c1, c2);
        break;
    }
    return buffer_offset;
}

int asercom_get_ground_sensors_binary_0xB3(int buffer_offset, BaseSequentialStream *uart_target) {
    int n, i = buffer_offset;
    for (int j = 0; j < 3; j++) {
        n = get_ground_prox(j);
        buffer[i++] = n & 0xff;
        buffer[i++] = n >> 8;
    }
    return i;
}

int asercom_get_proximity_sensors_binary0xB2(int buffer_offset, BaseSequentialStream *uart_target) {
    int i = buffer_offset, n;
    for (int j = 0; j < 8; j++) {
        n = e_get_calibrated_prox(j); // or ? n=e_get_prox(j);
        buffer[i++] = n & 0xff;
        buffer[i++] = n >> 8;
    }
    return i;
}

int asercom_get_ambient_light_binary_0xB1(int buffer_offset, BaseSequentialStream *uart_target) {
    int i = buffer_offset, n;
    for (int j = 0; j < 8; j++) {
        n = e_get_ambient_light(j);
        buffer[i++] = n & 0xff;
        buffer[i++] = n >> 8;
    }
    return i;
}

int asercom_set_motor_steps_binary_0xB0(int buffer_offset, BaseSequentialStream *uart_target) {
    char c1, c2;
    while (chSequentialStreamRead(uart_target, &c1, 1) == 0)
        ;
    while (chSequentialStreamRead(uart_target, &c2, 1) == 0)
        ;

    int positionl = (unsigned char)c1 + ((unsigned int)c2 << 8);

    while (chSequentialStreamRead(uart_target, &c1, 1) == 0)
        ;
    while (chSequentialStreamRead(uart_target, &c2, 1) == 0)
        ;

    int positionr = (unsigned char)c1 + ((unsigned int)c2 << 8);

    e_set_steps_left(positionl);
    e_set_steps_right(positionr);
    return buffer_offset;
}

int asercom_get_motor_steps_binary_0xAF(int buffer_offset, BaseSequentialStream *uart_target) {
    int i = buffer_offset, n;
    n = e_get_steps_left();
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;
    n = e_get_steps_right();
    buffer[i++] = n & 0xff;
    buffer[i++] = n >> 8;
    return i;
}

int asercom_get_temperature_binary_0x8C(int buffer_offset, BaseSequentialStream *uart_target) {
    buffer[buffer_offset++] = getTemperature();
    return buffer_offset;
}

/// @brief Buffer in which the uploaded bytecode is stored.
extern char *lua_bytecode_buffer;
extern uint16_t lua_bytecode_buffer_size;
extern binary_semaphore_t upload_complete;
uint32_t bytes_uploaded = 0;

/**
 * @brief Upload lua bytecode
 *
 * The code has to be uploaded into segments.
 *
 * | Magic Number | Total size | Offset  | Length  | Data | CRC-16  |
 * |--------------|------------|---------|---------|------|---------|
 * | 1B (1byte)   | 2 bytes    | 2 bytes | 2 bytes |  ... | 2 bytes |
 *
 * The magic number should always be `1B` (which is the lua bytecode magic number by the way), the total size should be
 * the length in bytes of the total message, offset should be the offset of the currently recieved packet compared to
 * the start of the packet in bytes, length should be the length in bytes of the current packet, data is tha payload,
 * and finally a CRC-16.
 *
 * The function will send '0' to the uart channel if the packet was recieved successfully, '1' if it fails.
 *
 * @param buffer_offset irrelevant
 * @param uart_target
 * @return int irrelevant
 */
int asercom_upload_lua_bytecode_0x1B(int buffer_offset, BaseSequentialStream *uart_target) {
    uint16_t total_bytes = 0;
    chSequentialStreamRead(uart_target, (uint8_t *)&total_bytes, 2);

    uint16_t offset = 0;
    chSequentialStreamRead(uart_target, (uint8_t *)&offset, 2);

    uint16_t packet_data_size = 0;
    chSequentialStreamRead(uart_target, (uint8_t *)&packet_data_size, 2);

    // TODO: remove this debugging:
    char s[128] = {0};
    sprintf(s, "got data packet: total: %d, packet: %d, offset: %d\r\n", total_bytes, packet_data_size, offset);
    chSequentialStreamWrite((BaseSequentialStream *)&SDU1, s, strlen(s));

    // only allocate once
    if (lua_bytecode_buffer_size < total_bytes) {
        lua_bytecode_buffer_size = total_bytes;
        lua_bytecode_buffer = chHeapAlloc(NULL, lua_bytecode_buffer_size); // NULL: use the general heap
        if (lua_bytecode_buffer == NULL) {
            buffer[buffer_offset++] = 1;
            return buffer_offset;
        }
    }

    // read stream directly into buffer to save memory
    chSequentialStreamRead(uart_target, (uint8_t *)&lua_bytecode_buffer[offset], packet_data_size);

    int16_t crc_16_checksum = 0;
    chSequentialStreamRead(uart_target, (uint8_t *)&crc_16_checksum, 2);

    // TODO: check crc

    bytes_uploaded += packet_data_size;
    if (bytes_uploaded == total_bytes) {
        chBSemSignal(&upload_complete); // signal main process to start lua VM
    }

    return buffer[buffer_offset++] = 0;
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
    // binary
    [0xf8] = asercom_get_all_sensors_binary_0xF8,
    [0xf7] = asercom_set_all_actuators_binary_0xF7,
    [0xf6] = asercom_set_rgb_led_binary_0xF6,
    [0xf5] = asercom_get_button_state_binary_0xF5,
    [0xf4] = asercom_get_microphones_binary_0xF4,
    [0xf3] = asercom_get_time_of_flight_binary_0xF3,
    [0xf2] = asercom_get_sd_card_state_binary_0xF2,
    [0xf1] = asercom_enable_disable_magnetometer_binary_0xF1,
    [0xf0] = asercom_set_proximity_state_binary_0xF0,
    [0xef] = asercom_enable_disable_time_of_flight_binary_0xEF,
    [0xee] = asercom_get_all_sensors_compact_binary_0xEE,
    [0x9f] = asercom_get_acceleration_raw_binary_0x9F,
    [0xa1] = asercom_get_acceleration_binary_0xA1,
    [0x9e] = asercom_get_battery_state_binary_0x9E,
    [0xbc] = asercom_set_motor_speed_binary_0xBC,
    [0xbb] = asercom_get_motor_speed_binary_0xBB,
    [0x99] = asercom_get_gyroscope_binary_0x99,
    [0xb7] = asercom_get_camera_image_binary_0xB7,
    [0xb4] = asercom_set_led_binary_0xB4,
    [0xb3] = asercom_get_ground_sensors_binary_0xB3,
    [0xb2] = asercom_get_proximity_sensors_binary0xB2,
    [0xb1] = asercom_get_ambient_light_binary_0xB1,
    [0xb0] = asercom_set_motor_steps_binary_0xB0,
    [0xaf] = asercom_get_motor_steps_binary_0xAF,
    [0x8c] = asercom_get_temperature_binary_0x8C,

    // Added in asercom3
    [0x1b] = asercom_upload_lua_bytecode_0x1B,
};

void run_asercom3(BaseSequentialStream *uart_target) {
    unsigned char c = 0;
    int index;

    char s[128] = {0};

    while (1) {
        index = 0;

        // blocking execution until we get something
        c = streamGet(uart_target);

        if (c > 0) {
            sprintf(s, "Got message with command %x.\r\n", c);
            chSequentialStreamWrite((BaseSequentialStream *)&SDU1, s, strlen(s));
            index = command_table[c](index, uart_target);
        } else {
            sprintf(s, "ChibiOS error %d. Could be MSG_TIMEOUT (-1) or MSG_RESET (-2) for example.\r\n", c);
            chSequentialStreamWrite((BaseSequentialStream *)&SDU1, s, strlen(s));
        }
    }
}
