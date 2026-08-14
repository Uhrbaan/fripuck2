#ifndef LUA_ROBOT_API_H
#define LUA_ROBOT_API_H

#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>

enum hooks {
    // sensors
    HOOK_TELEMETRY_PROXIMITY,
    HOOK_TELEMETRY_GROUND,
    HOOK_TELEMETRY_TOF,
    HOOK_TELEMETRY_BATTERY,
    HOOK_TELEMETRY_ENCODER,
    HOOK_TELEMETRY_IMU,
    HOOK_TELEMETRY_VOLUME,
    // commands
    HOOK_COMMAND_CUSTOM,
    // REPL (*can* be overwritten by user if they want, but managed internally if not.)
    HOOK_REPL,
    // TODO: commands
    HOOK_NUM,
};

int L_robot_on(lua_State* L);

extern const luaL_Reg robot_lib[];

#endif