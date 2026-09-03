#ifndef LUA_VM_H
#define LUA_VM_H

#include "robot_api.h"

// use the same identifiers for hooks or events
typedef enum hooks lua_event_type_t;

typedef struct {
    lua_event_type_t type;
    void* data;
    size_t data_len;
} lua_event_t;

int lua_vm_start(void);

#endif