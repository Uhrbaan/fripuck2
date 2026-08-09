#include "esp_log.h"

#include <stddef.h>
#include <string.h>

#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>

#include "hooks.h"

static const char* TAG = "HOOKS";

struct hook_entry {
    const char* hook_name;
    int lua_func_ref;  // reference index in the Lua Registry
};

static struct hook_entry hook_registry[HOOK_NUM];

static const char* hook_names[] = {
    [HOOK_TELEMETRY_PROXIMITY] = "telemetry:proximity",
    [HOOK_TELEMETRY_GROUND] = "telemetry:ground",
    [HOOK_TELEMETRY_TOF] = "telemetry:tof",
    [HOOK_TELEMETRY_BATTERY] = "telemetry:battery",
    [HOOK_TELEMETRY_ENCODER] = "telemetry:encoder",
    [HOOK_TELEMETRY_IMU] = "telemetry:imu",
    [HOOK_TELEMETRY_VOLUME] = "telemetry:volume",
    // TODO: commands
};

extern lua_State* L;

void hook_init() {
    for (int i = 0; i < HOOK_NUM; i++) {
        hook_registry[i] = (struct hook_entry){
            .hook_name = NULL,
            .lua_func_ref = LUA_NOREF,
        };
    }
}

int hook_name_to_num(const char* hook_name) {
    for (int i = 0; i < HOOK_NUM; i++) {
        if (strncmp(hook_name, hook_names[i], 40) == 0) {
            return i;
        }
    }
    return -1;
}

// TODO: automatically ref the ref
int register_hook(const char* hook_name, int lua_function_reference) {
    int hook_name_index = hook_name_to_num(hook_name);
    if (hook_name_index < 0) {
        ESP_LOGE(TAG, "Invalid hook name !");
        return 1;
    }

    if (hook_registry[hook_name_index].lua_func_ref == LUA_NOREF) {
        hook_registry[hook_name_index].hook_name =
            hook_names[hook_name_index];  // give the reference to the static string
        hook_registry[hook_name_index].lua_func_ref = lua_function_reference;
        ESP_LOGI(TAG, "Registered hook for `%s`.", hook_names[hook_name_index]);
        return 0;
    }

    ESP_LOGE(TAG, "Hook already set !");
    return 1;
}

// TODO: automatically unref the ref
int unregister_hook(const char* hook_name) {
    int hook_name_index = hook_name_to_num(hook_name);
    if (hook_name_index < 0) {
        ESP_LOGE(TAG, "Invalid hook name !");
        return 1;
    }

    hook_registry[hook_name_index].lua_func_ref = LUA_NOREF;
    ESP_LOGI(TAG, "Unregistered hook for `%s`.", hook_names[hook_name_index]);
    return 0;
}

int get_lua_hook_ref(const char* hook_name) {
    int hook_name_index = hook_name_to_num(hook_name);
    if (hook_name_index < 0) {
        ESP_LOGE(TAG, "Invalid hook name !");
        return LUA_NOREF;
    }

    return hook_registry[hook_name_index].lua_func_ref;
}

void trigger_hook(enum hooks hook_idx, float value) {
    if (hook_registry[hook_idx].lua_func_ref == LUA_NOREF) {
        ESP_LOGE(TAG, "Invalid hook name !");
        return;
    }

    // Retrieve the saved callback function from the registry onto the stack
    lua_rawgeti(L, LUA_REGISTRYINDEX, hook_registry[hook_idx].lua_func_ref);
    lua_pushnumber(L, value);
    if (lua_pcall(L, 1, 0, 0) != 0) {
        // Handle Lua error if the callback crashes
        printf("Error executing hook '%s': %s\n", hook_names[hook_idx], lua_tostring(L, -1));
        lua_pop(L, 1);  // Pop error message
    }
    ESP_LOGI(TAG, "Ran %s hook.", hook_names[hook_idx]);
}