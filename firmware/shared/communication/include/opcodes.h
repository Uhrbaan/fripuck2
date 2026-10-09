#ifndef SHARED_OPCODES_H
#define SHARED_OPCODES_H

#include <stdint.h>
#include <limits.h>
#include <stdbool.h>

#define OPCODES_BUF_MAX_SIZE UCHAR_MAX  // 255 bytes

typedef enum {
    ENABLE_ENABLE_OPCODE = 0x01,
    MOVEMENT_OPCODE = 0X02,
} command_opcodes;

#pragma pack(push, 1)
typedef struct {
    uint8_t opcode;
    uint32_t mask;
} enable_command_payload_t;
#pragma pack(pop)

struct move_trajectory_t {
    float speed;
    float distance;
    float radius;
    float accel;
    float decel;
    bool notify;
    bool synchronous;
};

#pragma pack(push, 1)
typedef struct {
    uint8_t opcode;
    struct move_trajectory_t options;
} move_trajectory_payload_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint8_t opcode;
    union {
        uint32_t mask;
        struct move_trajectory_t move_trajectory;
    };
} unified_instruction_payload_t;
#pragma pack(pop)

// Use explicit bitmasks instead of C bitfields for cross-compiler safety
#define MASK_MODE_SELECTOR (1UL << 0)
#define MASK_IR_RECEIVER (1UL << 1)
#define MASK_BATTERY (1UL << 2)
// 6 LEDs occupying bits 3, 4, 5, 6, 7, 8
#define MASK_LED_1 (1UL << 3)
#define MASK_LED_3 (1UL << 4)
#define MASK_LED_5 (1UL << 5)
#define MASK_LED_7 (1UL << 6)
#define MASK_BODY_LED (1UL << 7)
#define MASK_FRONT_LED (1UL << 8)
#define MASK_LED_ALL (MASK_LED_1 | MASK_LED_3 | MASK_LED_5 | MASK_LED_7 | MASK_BODY_LED | MASK_FRONT_LED)

#define MASK_PROXIMITY (1UL << 9)
#define MASK_TOF (1UL << 10)
#define MASK_IMU (1UL << 11)
#define MASK_CAMERA (1UL << 12)
#define MASK_MOTORS (1UL << 13)
#define MASK_GROUND (1UL << 14)

#endif