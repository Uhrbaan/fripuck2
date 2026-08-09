#ifndef LUAVM_HOOKS_H
#define LUAVM_HOOKS_H

enum hooks {
    HOOK_TELEMETRY_PROXIMITY,
    HOOK_TELEMETRY_GROUND,
    HOOK_TELEMETRY_TOF,
    HOOK_TELEMETRY_BATTERY,
    HOOK_TELEMETRY_ENCODER,
    HOOK_TELEMETRY_IMU,
    HOOK_TELEMETRY_VOLUME,
    // TODO: commands
    HOOK_NUM,
};

void hook_init();
int hook_name_to_num(const char* hook_name);
int register_hook(const char* hook_name, int lua_function_reference);
int unregister_hook(const char* hook_name);
int get_lua_hook_ref(const char* hook_name);
void trigger_hook(enum hooks hook_idx, float value);

#endif