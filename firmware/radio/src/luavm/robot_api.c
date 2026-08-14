#include "esp_log.h"

#include "robot_api.h"

#include "all_lua_types.h"

static const char* TAG = "ROBOT API";

typedef int (*hook_register_fn)(lua_State*, int);

static const char* hook_names[HOOK_NUM] = {
    // sensors
    [HOOK_TELEMETRY_PROXIMITY] = "telemetry:proximity",
    [HOOK_TELEMETRY_GROUND] = "telemetry:ground",
    [HOOK_TELEMETRY_TOF] = "telemetry:tof",
    [HOOK_TELEMETRY_BATTERY] = "telemetry:battery",
    [HOOK_TELEMETRY_ENCODER] = "telemetry:encoder",
    [HOOK_TELEMETRY_IMU] = "telemetry:imu",
    [HOOK_TELEMETRY_VOLUME] = "telemetry:volume",
    // commands
    [HOOK_COMMAND_CUSTOM] = "custom:",
    // repl
    [HOOK_REPL] = "repl",  // TODO: still have to implement
    // TODO: commands
};

static const hook_register_fn hook_register_functions[] = {
    [HOOK_TELEMETRY_PROXIMITY] = NULL,        [HOOK_TELEMETRY_GROUND] = register_ground_hook,
    [HOOK_TELEMETRY_TOF] = register_tof_hook, [HOOK_TELEMETRY_BATTERY] = NULL,
    [HOOK_TELEMETRY_ENCODER] = NULL,          [HOOK_TELEMETRY_IMU] = NULL,
    [HOOK_TELEMETRY_VOLUME] = NULL,           [HOOK_COMMAND_CUSTOM] = register_custom_hook,
    // TODO: commands
};

/**
 * This function register a hook to the lua VM. If the hook is created, as soon as a data piece is detected, the
 * provided lua function is called.
 *
 * The function signature is:
 * fun(hook_name: string, function: fun(value))
 */
int L_robot_on(lua_State* L) {
    const char* hook_name = luaL_checkstring(L, 1);
    ESP_LOGI(TAG, "Trying to register `%s`", hook_name);

    enum hooks hook_id = -1;
    for (int i = 0; i < HOOK_NUM; i++) {
        if (strncmp(hook_name, hook_names[i], 40) == 0) {
            hook_id = i;
            break;
        }
    }

    // for custom commands, we only look if the start is 'custom:'
    if (hook_id == -1 &&
        strncmp(hook_name, hook_names[HOOK_COMMAND_CUSTOM], strlen(hook_names[HOOK_COMMAND_CUSTOM])) == 0) {
        hook_id = HOOK_COMMAND_CUSTOM;
    }
    if (hook_id == -1) {
        return luaL_error(L, "Invalid hook !");
    }

    // Takes care of the function argument (argument 2)
    ESP_LOGI(TAG, "Attempting to register %s.", hook_name);
    if (hook_register_functions[hook_id](L, 2) != 0) {
        return luaL_error(L, "Could not register hook.");
    }

    return 0;
}

const luaL_Reg robot_lib[] = {{"on", L_robot_on}, {NULL, NULL}};