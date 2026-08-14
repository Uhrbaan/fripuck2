#ifndef LUA_VM_CUSTOM_H
#define LUA_VM_CUSTOM_H

#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>

#include "commands_reader.h"

void custom_type_init(lua_State* L);
int register_custom_hook(lua_State* L, int narg);
void trigger_custom_hook(const char* topic, const char* payload, size_t payload_length);

#endif