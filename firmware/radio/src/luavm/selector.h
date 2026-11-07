#ifndef SELECTOR_LUA_T_H
#define SELECTOR_LUA_T_H

#include "lua.h"

void selector_type_init(lua_State* L);
int register_selector_hook(lua_State* L, int narg);
void trigger_selector_hook(const uint8_t* new_value);
void execute_selector_hook(lua_State* L);
int push_selector_data(lua_State* L);

#endif