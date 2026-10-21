#include "esp_log.h"

#include "robot_api.h"

#include "all_lua_types.h"
#include "commands/commands.h"
#include "opcodes.h"

static const char* TAG = "ROBOT API";
// FIXME: actually calculate what the values are

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

int L_robot_set_modules(lua_State* L) {
    // 1. Ensure the first argument is a table
    luaL_checktype(L, 1, LUA_TTABLE);

    uint32_t enable_mask = 0;
    uint32_t disable_mask = 0;

    // 2. Iterate through the table key-value pairs
    lua_pushnil(L);  // First key for lua_next
    while (lua_next(L, 1) != 0) {
        // Stack layout: table at 1, key at -2, value at -1

        // Ensure key is a string
        if (lua_type(L, -2) != LUA_TSTRING) {
            return luaL_error(L, "Invalid table key: keys must be strings");
        }

        const char* key = lua_tostring(L, -2);
        bool value = lua_toboolean(L, -1);
        uint32_t target_module_mask = 0;

        // 3. Map keys to the corresponding hardware masks
        if (strcmp(key, "mode_selector") == 0) {
            target_module_mask = MASK_MODE_SELECTOR;
        } else if (strcmp(key, "ir_receiver") == 0) {
            target_module_mask = MASK_IR_RECEIVER;
        } else if (strcmp(key, "battery") == 0) {
            target_module_mask = MASK_BATTERY;
        } else if (strcmp(key, "proximity") == 0) {
            target_module_mask = MASK_PROXIMITY;
        } else if (strcmp(key, "ring_led_1") == 0) {
            target_module_mask = MASK_LED_1;
        } else if (strcmp(key, "ring_led_3") == 0) {
            target_module_mask = MASK_LED_3;
        } else if (strcmp(key, "ring_led_5") == 0) {
            target_module_mask = MASK_LED_5;
        } else if (strcmp(key, "ring_led_7") == 0) {
            target_module_mask = MASK_LED_7;
        } else if (strcmp(key, "body_led") == 0) {
            target_module_mask = MASK_BODY_LED;
        } else if (strcmp(key, "front_led") == 0) {
            target_module_mask = MASK_FRONT_LED;
        } else if (strcmp(key, "tof") == 0) {
            target_module_mask = MASK_TOF;
        } else if (strcmp(key, "imu") == 0) {
            target_module_mask = MASK_IMU;
        } else if (strcmp(key, "camera") == 0) {
            target_module_mask = MASK_CAMERA;
        } else if (strcmp(key, "motors") == 0) {
            target_module_mask = MASK_MOTORS;
        } else if (strcmp(key, "ground") == 0) {
            target_module_mask = MASK_GROUND;
        } else {
            // Error out on unexpected/invalid key names
            return luaL_error(L, "Unknown module option: '%s'", key);
        }

        // Apply the resolved mask to either the enable or disable bucket
        if (value) {
            enable_mask |= target_module_mask;
        } else {
            disable_mask |= target_module_mask;
        }

        // Pop the value, leave key for the next lua_next call
        lua_pop(L, 1);
    }

    // Apply the accumulated changes additively
    update_system_state(enable_mask, disable_mask);

    // Return 0 for no return values
    return 0;
}

const luaL_Reg robot_lib[] = {{"on", L_robot_on}, {"set_modules", L_robot_set_modules}, {NULL, NULL}};