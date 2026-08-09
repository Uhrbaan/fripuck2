#include "esp_log.h"

#include "robot_api.h"
#include "hooks.h"

/**
 * This function register a hook to the lua VM. If the hook is created, as soon as a data piece is detected, the
 * provided lua function is called.
 *
 * The function signature is:
 * fun(hook_name: string, function: fun(value))
 */
int L_robot_on(lua_State* L) {
    const char* hook_name = luaL_checkstring(L, 1);

    // If the funciton is nil, dereference the function from hooks
    if (lua_type(L, 2) == LUA_TNIL) {
        int ref = get_lua_hook_ref(hook_name);
        unregister_hook(hook_name);
        luaL_unref(L, LUA_REGISTRYINDEX, ref);

        lua_pushboolean(L, true);
        return 1;
    }

    // Fail here if no function is given.
    luaL_checktype(L, 2, LUA_TFUNCTION);

    // Push the function (argument #2) to the top of the stack so luaL_ref can pop it
    lua_pushvalue(L, 2);

    // Storing the function in LUA_REGISTRYINDEX prevents it from being garbage collected
    int ref = luaL_ref(L, LUA_REGISTRYINDEX);

    if (register_hook(hook_name, ref) != 0) {
        // There was not enough space in the registry, remove it.
        luaL_unref(L, LUA_REGISTRYINDEX, ref);
        return luaL_error(L, "Invalid hook!");
    }

    lua_pushboolean(L, true);

    return 1;  // 0 return values
}

const luaL_Reg robot_lib[] = {{"on", L_robot_on}, {NULL, NULL}};