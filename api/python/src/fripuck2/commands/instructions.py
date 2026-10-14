from enum import IntFlag

class ModuleMask(IntFlag):
    NONE          = 0
    MODE_SELECTOR = 1 << 0
    IR_RECEIVER   = 1 << 1
    BATTERY       = 1 << 2
    LED_1         = 1 << 3
    LED_3         = 1 << 4
    LED_5         = 1 << 5
    LED_7         = 1 << 6
    BODY_LED      = 1 << 7
    FRONT_LED     = 1 << 8
    PROXIMITY     = 1 << 9
    TOF           = 1 << 10
    IMU           = 1 << 11
    CAMERA        = 1 << 12
    MOTORS        = 1 << 13
    GROUND        = 1 << 14