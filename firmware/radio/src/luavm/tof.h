#ifndef TOF_LUA_T_H
#define TOF_LUA_T_H

#include <lua.h>
#include "sensors_reader.h"

void tof_type_init(lua_State* L);
int register_tof_hook(lua_State* L, int narg);
void trigger_tof_hook(const FripuckProtocol_Sensors_TofData_t* new_value);
void execute_tof_hook(lua_State* L);

#endif