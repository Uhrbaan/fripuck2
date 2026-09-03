#ifndef SRC_COMMANDS_H
#define SRC_COMMANDS_H

struct enable_modules_params {
    bool enable_mode_selector;
    bool enable_ir_receiver;
    bool enable_battery;
    bool enable_proximity;
    bool enable_ring_led_1;
    bool enable_ring_led_3;
    bool enable_ring_led_5;
    bool enable_ring_led_7;
    bool enable_body_led;
    bool enable_front_led;
    bool enable_tof;
    bool enable_imu;
    bool enable_camera;
    bool enable_motors;
    bool enable_ground;
};

void command_receive(const uint8_t* data, uint32_t length);
void command_enable_modules(struct enable_modules_params p);
#endif