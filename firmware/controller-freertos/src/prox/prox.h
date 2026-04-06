#ifndef SRC_PROX_H
#define SRC_PROX_H

#include "proximity.h"
#include "sensors_builder.h"

void prox_initialize_lock(void);
void prox_insert_callback(struct Uint16Array8 *s);
void pack_prox_to_vector(flatcc_builder_t *builder);

#endif