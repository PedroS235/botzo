#ifndef SERVO_MANAGER_H
#define SERVO_MANAGER_H

#include <hardware/i2c.h>
#include <stdint.h>
typedef struct {
    int min_angle;
    int max_angle;
    uint16_t min_pulse;
    uint16_t max_pulse;
    uint16_t frequency;
} ServoMgrConfig;

void servo_mgr_init(i2c_inst_t *i2c, uint8_t pca9685_addr);
void servo_mgr_move_to(uint8_t index, float angle);
void servo_mgr_move_pulse(uint8_t index, uint16_t pulse);

#endif  // !SERVO_MANAGER_H
