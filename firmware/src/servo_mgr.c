#include "servo_mgr.h"

#include <stdint.h>

#include "drivers/pca9685/pca9685.h"

void servo_mgr_init(i2c_inst_t *i2c, uint8_t pca9685_addr) {
    PCA9685Config_t pca9685_config = {.freq = 50};
    pca9685_init(i2c, pca9685_addr, &pca9685_config);
}

void servo_mgr_move_to(uint8_t index, float angle) {
    uint16_t pulse_width_us = 500 + (angle * 2000) / 270;
    pca9685_set_pulse_width(index, pulse_width_us);
}

void servo_mgr_move_pulse(uint8_t index, uint16_t pulse) {
    pca9685_set_pulse_width(index, pulse);
}
