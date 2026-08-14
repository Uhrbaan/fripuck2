#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "custom.h"

static const char* TAG = "CUSTOM LTYPE";

struct topic_lfn_pair {
    char* topic;
    int lua_func_ref;
};

#define MAX_CUSTOM_HOOKS 20
static struct topic_lfn_pair topic_function_pairs[MAX_CUSTOM_HOOKS] = {0};

void custom_type_init(lua_State* L) {
    for (int i = 0; i < MAX_CUSTOM_HOOKS; i++) {
        topic_function_pairs[i].lua_func_ref = LUA_NOREF;
        topic_function_pairs[i].topic = NULL;
    }
}

static lua_State* L = NULL;

int register_custom_hook(lua_State* Lstate, int narg) {
    L = Lstate;
    const char* hook_name = luaL_checkstring(L, 1);
    char* colon = strchr(hook_name, ':');

    if (colon == NULL || colon[1] == '\0') {
        ESP_LOGE(TAG, "Invalid hook name format. Expected 'custom:topic'");
        return 1;
    }

    const char* topic = &colon[1];

    // Unregister logic
    if (lua_type(L, narg) == LUA_TNIL) {
        for (int i = 0; i < MAX_CUSTOM_HOOKS; i++) {
            if (topic_function_pairs[i].topic != NULL && strcmp(topic_function_pairs[i].topic, topic) == 0) {
                luaL_unref(L, LUA_REGISTRYINDEX, topic_function_pairs[i].lua_func_ref);
                topic_function_pairs[i].lua_func_ref = LUA_NOREF;

                free(topic_function_pairs[i].topic);
                topic_function_pairs[i].topic = NULL;

                ESP_LOGI(TAG, "Successfully unregistered custom:%s", topic);
                return 0;
            }
        }
        ESP_LOGW(TAG, "custom:%s wasn't registered in the first place", topic);
        return 0;
    }

    if (lua_type(L, narg) != LUA_TFUNCTION) {
        ESP_LOGE(TAG, "Provided argument is not a function, nor nil.");
        return 1;
    }

    int empty_slot = -1;
    for (int i = 0; i < MAX_CUSTOM_HOOKS; i++) {
        if (topic_function_pairs[i].lua_func_ref != LUA_NOREF) {
            if (topic_function_pairs[i].topic != NULL && strcmp(topic_function_pairs[i].topic, topic) == 0) {
                // Topic already exists, let's reuse this slot (overwrite)
                empty_slot = i;
                luaL_unref(L, LUA_REGISTRYINDEX, topic_function_pairs[i].lua_func_ref);  // unref old func
                break;
            }
        } else if (empty_slot == -1) {
            // Keep track of the first available empty slot just in case
            empty_slot = i;
        }
    }

    if (empty_slot != -1) {
        int i = empty_slot;

        // Only allocate if it's a completely new topic
        if (topic_function_pairs[i].topic == NULL || strcmp(topic_function_pairs[i].topic, topic) != 0) {
            char* p = strdup(topic);
            if (p == NULL) {
                ESP_LOGE(TAG, "Could not allocate space for topic.");
                return 1;
            }
            free(topic_function_pairs[i].topic);
            topic_function_pairs[i].topic = p;
        }

        lua_pushvalue(L, narg);
        topic_function_pairs[i].lua_func_ref = luaL_ref(L, LUA_REGISTRYINDEX);
        ESP_LOGI(TAG, "Successfully registered custom:%s !", topic);
        return 0;
    }

    ESP_LOGE(TAG, "There is no more space for additional hooks. Increase MAX_CUSTOM_HOOKS.");
    return 1;
}

void trigger_custom_hook(const char* topic, const char* payload, size_t payload_length) {
    for (int i = 0; i < MAX_CUSTOM_HOOKS; i++) {
        if (topic_function_pairs[i].topic != NULL && topic_function_pairs[i].lua_func_ref != LUA_NOREF &&
            strcmp(topic_function_pairs[i].topic, topic) == 0) {
            if (payload != NULL) {
                lua_rawgeti(L, LUA_REGISTRYINDEX, topic_function_pairs[i].lua_func_ref);
                lua_pushlstring(L, payload, payload_length);
                if (lua_pcall(L, 1, 0, 0) != 0) {
                    ESP_LOGE(TAG, "Error executing ground hook: %s\n", lua_tostring(L, -1));
                    lua_pop(L, 1);  // Pop error message
                }
            } else {
                ESP_LOGE(TAG, "Failed to allocate memory for payload");
            }
        }
    }
}