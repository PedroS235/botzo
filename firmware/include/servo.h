#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#include "error.h"
#include "pico.h"

typedef enum { SERVO_STATE_IDLE, SERVO_STATE_MOVING, SERVO_STATE_ERROR } servo_state_e;

typedef struct {
    float a, b, c;  // count = a*angle² + b*angle + c (PCA9685 12-bit: 0-4095)
} servo_calib_t;

typedef struct {
    uint8_t channel;
    servo_calib_t calib;
    float min_angle, max_angle;
    uint16_t min_count, max_count;  // PCA9685 counter limits (0-4095)

    float current_angle;
    float target_angle;
    uint16_t current_count;  // Last commanded counter value
    servo_state_e state;

    float start_angle;
    uint64_t interp_start_us;
    uint64_t interp_duration_us;

    float us_per_count;  // Precise conversion factor from PCA9685

    bool initialized;
} servo_t;

typedef struct {
    uint8_t channel;
    servo_calib_t calib;
    float min_angle, max_angle;
    uint16_t min_count, max_count;  // PCA9685 counter limits
} servo_config_t;

ErrorCode_e servo_init(servo_t *servo, const servo_config_t *config);
ErrorCode_e servo_set_target(servo_t *servo, float angle, uint32_t duration_ms);
ErrorCode_e servo_update(servo_t *servo);  // Called by scheduler
ErrorCode_e servo_angle_to_count(const servo_t *servo, float angle, uint16_t *count);
ErrorCode_e servo_is_moving(const servo_t *servo);
ErrorCode_e servo_stop(servo_t *servo);

// Direct control (for testing/development)
ErrorCode_e servo_set_count_direct(servo_t *servo, uint16_t count);
ErrorCode_e servo_set_pulse_direct(servo_t *servo, uint16_t pulse_us);

// Conversion helpers
uint16_t servo_pulse_to_count(const servo_t *servo, uint16_t pulse_us);
uint16_t servo_count_to_pulse(const servo_t *servo, uint16_t count);
ErrorCode_e servo_get_angle(const servo_t *servo, float *angle);

#endif  // !SERVO_H
