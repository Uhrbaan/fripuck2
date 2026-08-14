#ifndef LUA_VM_GROUND_H
#define LUA_VM_GROUND_H

#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>

#include "sensors_reader.h"

void ground_type_init(lua_State* L);
int register_ground_hook(lua_State* Lstate, int narg);
void trigger_ground_hook(const FripuckProtocol_Sensors_GroundData_t* new_value);
void execute_ground_hook(lua_State* L);

#endif