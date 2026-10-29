#pragma once

#include <stdint.h>

typedef struct {
    int32_t prev_sample;
    int32_t prev_index;
} ADPCM_State;

uint8_t ADPCM_EncodeSample(int32_t sample, ADPCM_State* state);