#pragma once

#include <stdint.h>
#include "adpcm.h"

// https://wiki.multimedia.cx/index.php?title=IMA_ADPCM
static const int16_t adpcm_step_table[] = {
    7,    8,     9,     10,    11,    12,    13,    14,    16,    17,    19,    21,    23,    25,   28,
    31,   34,    37,    41,    45,    50,    55,    60,    66,    73,    80,    88,    97,    107,  118,
    130,  143,   157,   173,   190,   209,   230,   253,   279,   307,   337,   371,   408,   449,  494,
    544,  598,   658,   724,   796,   876,   963,   1060,  1166,  1282,  1411,  1552,  1707,  1878, 2066,
    2272, 2499,  2749,  3024,  3327,  3660,  4026,  4428,  4871,  5358,  5894,  6484,  7132,  7845, 8630,
    9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767};

static const int8_t adpcm_index_table[] = {-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8};

uint8_t ADPCM_EncodeSample(int32_t sample, ADPCM_State* state) {
    int32_t diff = sample - state->prev_sample;
    uint8_t code = 0;
    int32_t step = adpcm_step_table[state->prev_index];
    int32_t predicted_diff = step >> 3;

    if (diff < 0) {
        code = 8;
        diff = -diff;
    }

    if (diff >= step) {
        code |= 4;
        diff -= step;
        predicted_diff += step;
    }
    step >>= 1;
    if (diff >= step) {
        code |= 2;
        diff -= step;
        predicted_diff += step;
    }
    step >>= 1;
    if (diff >= step) {
        code |= 1;
        predicted_diff += step;
    }

    if (code & 8) {
        state->prev_sample -= predicted_diff;
    } else {
        state->prev_sample += predicted_diff;
    }

    if (state->prev_sample > 32767)
        state->prev_sample = 32767;
    else if (state->prev_sample < -32768)
        state->prev_sample = -32768;

    state->prev_index += adpcm_index_table[code];
    if (state->prev_index < 0)
        state->prev_index = 0;
    else if (state->prev_index > 88)
        state->prev_index = 88;

    return code;
}