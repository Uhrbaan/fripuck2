#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

#include "aseba_vm/aseba_bridge.h"
#include "aseba_vm/aseba_can_interface.h"
#include "aseba_vm/aseba_node.h"
#include "aseba_vm/skel_user.h"
#include "audio/audio_thread.h"
#include "audio/microphone.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "behaviors.h"
#include "button.h"
#include "camera/camera.h"
#include "cmd.h"
#include "communication.h"
#include "config_flash_storage.h"
#include "epuck1x/Asercom.h"
#include "epuck1x/Asercom2.h"
#include "epuck1x/a_d/advance_ad_scan/e_acc.h"
#include "epuck1x/motor_led/advance_one_timer/e_led.h"
#include "epuck1x/uart/e_uart_char.h"
#include "epuck1x/utility/utility.h"
#include "exti.h"
#include "fat.h"
#include "i2c_bus.h"
#include "ir_remote.h"
#include "ircom/ircom.h"
#include "ircom/ircomMessages.h"
#include "ircom/ircomReceive.h"
#include "ircom/ircomSend.h"
#include "ircom/transceiver.h"
#include "leds.h"
#include "memory_protection.h"
#include "motors.h"
#include "msgbus/messagebus.h"
#include "sdio.h"
#include "selector.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/battery_level.h"
#include "sensors/ground.h"
#include "sensors/imu.h"
#include "sensors/mpu9250.h"
#include "sensors/proximity.h"
#include "serial_comm.h"
#include "spi_comm.h"
#include "uc_usage.h"
#include "usbcfg.h"
#include <main.h>

#include "lua/luavm.h"

#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

parameter_namespace_t parameter_root, aseba_ns;

static THD_WORKING_AREA(selector_thd_wa, 2048);

static bool load_config(void) {
    extern uint32_t _config_start;

    return config_load(&parameter_root, &_config_start);
}

// Declares a thread function hiding eventual compiler-specific keywords.
// <https://chibios.org/dokuwiki/doku.php?id=chibios:documentation:books:rt:kernel_threading>
static THD_FUNCTION(selector_thd, arg) {
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    uint8_t temp_rx = 0;

    if (cam_advanced_config(FORMAT_COLOR, 0, 0, 640, 480, SUBSAMPLING_X4, SUBSAMPLING_X4) != MSG_OK) {
        set_led(LED1, 1);
    }
    cam_set_exposure(512, 0); // Fix the exposure to have a stable framerate.

    dcmi_set_capture_mode(CAPTURE_ONE_SHOT);

    if (dcmi_prepare() < 0) {
        set_led(LED5, 1);
    }

    spi_image_transfer_enable();

    mpu9250_magnetometer_setup();

    // Flush the uart input to avoid interpreting garbage as real commands.
    while (chnReadTimeout(&SD3, (uint8_t *)&temp_rx, 1, MS2ST(1) > 0)) {
        chThdSleepMilliseconds(1);
    }

    run_asercom2();
}

int main(void) {
    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    parameter_namespace_declare(&parameter_root, NULL, NULL);

    // Init the peripherals.
    clear_leds();
    set_body_led(0);
    set_front_led(0);
    usb_start();
    dcmi_start();
    cam_start();
    motors_init();
    proximity_start(FAST_UPDATE);
    battery_level_start();
    dac_start();
    exti_start();
    imu_start();
    ir_remote_start();
    spi_comm_start();
    VL53L0X_start();
    serial_start();
    mic_start(NULL);
    sdio_start();
    playMelodyStart();
    playSoundFileStart();
    ground_start();
    behaviors_start();

    // Initialise Aseba system, declaring parameters
    parameter_namespace_declare(&aseba_ns, &parameter_root, "aseba");
    aseba_declare_parameters(&aseba_ns);

    /* Load parameter tree from flash. */
    load_config();

    /* Start AsebaCAN. Must be after config was loaded because the CAN id
     * cannot be changed at runtime. */
    aseba_vm_init();
    aseba_can_start(&vmState);

    chThdCreateStatic(selector_thd_wa, sizeof(selector_thd_wa), NORMALPRIO, selector_thd, NULL);
    // lua_start_vm();

    /* Infinite loop. */
    while (1) {
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) { chSysHalt("Stack smashing detected"); }
