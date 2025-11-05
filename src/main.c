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

#include "lua/lua.h"
#include "lua/lauxlib.h"
#include "lua/lualib.h"

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

static int Lenable_four_leds(lua_State *L) {
	// check the first integer passed as argument
	int bits = luaL_checkinteger(L, 1);

	// clear_leds();
	for (int i=0; i<4; i++) {
		if (bits & 1 << i) {
			set_led(i, 1);
		} else {
			set_led(i, 0);
		}
	}

	// return the number of results, 0 in this case 
	return 0;
}

static int Lsleep_seconds(lua_State *L) {
	int seconds = luaL_checkinteger(L, 1);
	chThdSleepSeconds(seconds);
	return 0;
}

// Declares a thread function hiding eventual compiler-specific keywords. 
// <https://chibios.org/dokuwiki/doku.php?id=chibios:documentation:books:rt:kernel_threading>
static THD_FUNCTION(selector_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

	lua_State *L = luaL_newstate();
	lua_pushcfunction(L, Lenable_four_leds);
    lua_setglobal(L, "enableLEDs");
	lua_pushcfunction(L, Lsleep_seconds);
	lua_setglobal(L, "sleep");

	const char script[] =
		"local led = 0\n"
		"while true do\n"
		"	led = led + 1\n"
		"	if led > 15 then\n"
		"		led = 0\n"
		"	end\n"
		"	enableLEDs(led)\n"
		"	sleep(1)\n"
		"end\n";

	// blocking
	luaL_dostring(L, script);

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
