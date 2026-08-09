#ifndef LUA_ROBOT_API_H
#define LUA_ROBOT_API_H

#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>

int L_robot_on(lua_State* L);

extern const luaL_Reg robot_lib[];

#endif