#ifndef FB_INIT_H
#define FB_INIT_H

#include "flatcc/flatcc_builder.h"

int init_flatbuffers(flatcc_builder_t* builder);
void* get_final_buffer(flatcc_builder_t* builder, size_t* out_size);
void reset_emitter(flatcc_builder_t* builder);

#endif