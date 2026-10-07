#pragma once

#include <stddef.h>
#include <stdint.h>
void prepare_and_send_instruction(uint8_t* data, uint16_t length);
int start_instruction_handler(size_t queue_length);