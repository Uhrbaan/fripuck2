#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

#include "aseba_vm/aseba_node.h"
#include "aseba_vm/skel_user.h"
#include "aseba_vm/aseba_can_interface.h"
#include "aseba_vm/aseba_bridge.h"
#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "audio/microphone.h"
#include "camera/camera.h"
#include "epuck1x/Asercom.h"
#include "epuck1x/Asercom2.h"
#include "epuck1x/a_d/advance_ad_scan/e_acc.h"
#include "epuck1x/motor_led/advance_one_timer/e_led.h"
#include "epuck1x/utility/utility.h"
#include "sensors/battery_level.h"
#include "sensors/ground.h"
#include "sensors/imu.h"
#include "sensors/mpu9250.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "behaviors.h"
#include "button.h"
#include "cmd.h"
#include "config_flash_storage.h"
#include "exti.h"
#include "fat.h"
#include "i2c_bus.h"
#include "ir_remote.h"
#include "leds.h"
#include <main.h>
#include "memory_protection.h"
#include "motors.h"
#include "sdio.h"
#include "selector.h"
#include "serial_comm.h"
#include "spi_comm.h"
#include "usbcfg.h"
#include "communication.h"
#include "uc_usage.h"
#include "ircom/ircom.h"
#include "ircom/ircomReceive.h"
#include "ircom/ircomMessages.h"
#include "ircom/ircomSend.h"
#include "ircom/transceiver.h"

#include "lua/src/lua.h"
#include "lua/src/lauxlib.h"
#include "lua/src/lualib.h"

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

parameter_namespace_t parameter_root, aseba_ns;

static THD_WORKING_AREA(selector_thd_wa, 2048);

static bool load_config(void)
{
    extern uint32_t _config_start;

    return config_load(&parameter_root, &_config_start);
}

static int enable_four_leds(lua_State *L) {
	// check the first integer passed as argument
	int bits = luaL_checkinteger(L, 1);

	if (bits & 1 << 0) {
		set_led(LED1, 1);
	}
	if (bits & 1 << 1) {
		set_led(LED3, 1);
	}
	if (bits & 1 << 2) {
		set_led(LED5, 1);
	}
	if (bits & 1 << 3) {
		set_led(LED7, 1);
	}

	// return the number of results, 0 in this case 
	return 0;
}

// Declares a thread function hiding eventual compiler-specific keywords. 
// <https://chibios.org/dokuwiki/doku.php?id=chibios:documentation:books:rt:kernel_threading>
static THD_FUNCTION(selector_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);
    uint8_t stop_loop = 0;
    systime_t time;

    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;
    int16_t leftSpeed = 0, rightSpeed = 0;
    int16_t prox_values_temp[8];

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;

    uint16_t prox_thr = 1000;

    uint8_t hw_test_state = 0;
    uint8_t *img_buff_ptr;
    uint16_t r = 0, g = 0, b = 0;
    uint8_t rgb_state = 0, rgb_counter = 0;
    uint16_t melody_state = 0, melody_counter = 0;
    int8_t cam_error = 0;

    uint8_t magneto_state = 0;
    
    uint8_t demo15_state = 0;
    uint8_t temp_rx = 0;

	uint8_t rab_addr = 0x20;
	uint8_t rab_state = 0;
	int8_t i2c_err = 0;
	uint8_t regValue[2] = {0};
	uint16_t rab_data = 0;
	double rab_bearing = 0.0;
	uint16_t rab_range = 0;
	uint16_t rab_sensor = 0;
	uint8_t rab_buff[35];
	uint16_t rab_tx_data = 0;
	uint8_t rab_counter = 0;

	uint8_t back_and_forth_state = 0;
	float turn_angle_rad = 0.0;
	uint8_t led_animation_state = 0;
	uint32_t led_animation_count = 0;

	uint8_t wav_volume = 20;
	uint8_t wav_play_state = 0;

	double heading = 0.0;
	float mag_values[3];

	//debug variables
	//static float geo_offsets[3] = {0, 0, 0};
	//static float geo_offsets_max[3] = {-1000, -1000, -1000};
	//static float geo_offsets_min[3] = {1000, 1000, 1000};
	//static float compass = 0;

	// calibrate_acc();
	// calibrate_gyro();
	// calibrate_ir();

	lua_State *L = luaL_newstate();
	lua_pushcfunction(L, enable_four_leds);
    lua_setglobal(L, "enableLEDs");

	luaL_dostring(L, "enableLEDs(3)");

	// clean the lua VM 
	lua_close(L);
}

int main(void)
{

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

    /* Infinite loop. */
    while (1) {
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
