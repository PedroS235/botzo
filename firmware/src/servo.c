#include "servo.h"

#include <hardware/timer.h>
#include <pico/platform/compiler.h>
#include <pico/time.h>
#include <pico/types.h>
#include <stdint.h>
#include <stdio.h>

#include "drivers/pca9685/pca9685.h"
#include "error.h"

ErrorCode_e servo_init(servo_t *servo, const servo_config_t *config) {
    if (!servo) return ERR_INVALID_PARAM;

    servo->channel = config->channel;
    servo->min_angle = config->min_angle;
    servo->max_angle = config->max_angle;
    servo->min_count = config->min_count;
    servo->max_count = config->max_count;
    servo->calib = config->calib;

    // Get precise conversion factor from PCA9685
    ErrorCode_e err = pca9685_get_us_per_count(&servo->us_per_count);
    if (err != OK) {
        return err;
    }

    servo->current_angle = 0;
    servo->target_angle = 0;
    servo->current_count = servo->min_count;
    servo->state = SERVO_STATE_IDLE;

    servo->start_angle = 0;
    servo->interp_start_us = 0;
    servo->interp_duration_us = 0;

    servo->initialized = true;

    return OK;
}

ErrorCode_e servo_set_target(servo_t *servo, float angle, uint32_t duration_ms) {
    if (!servo || !servo->initialized || angle > servo->max_angle ||
        angle < servo->min_angle)
        return ERR_INVALID_PARAM;

    servo->target_angle = angle;

    if (duration_ms == 0) {
        servo->current_angle = angle;
        servo_angle_to_count(servo, angle, &servo->current_count);
        return pca9685_set_pwm(servo->channel, 0, servo->current_count);
    }

    servo->start_angle = servo->current_angle;
    servo->interp_duration_us = duration_ms * 1000;
    servo->interp_start_us = get_absolute_time();
    servo->state = SERVO_STATE_MOVING;

    return OK;
}

ErrorCode_e servo_update(servo_t *servo) {
    if (!servo || !servo->initialized) return ERR_INVALID_PARAM;

    if (servo->state != SERVO_STATE_MOVING) return OK;

    uint64_t now = time_us_64();
    int64_t elapsed = absolute_time_diff_us(servo->interp_start_us, now);
    if (elapsed >= servo->interp_duration_us) {
        servo->current_angle = servo->target_angle;
        servo->state = SERVO_STATE_IDLE;
    } else {
        float progress = elapsed / (float)servo->interp_duration_us;
        servo->current_angle =
            servo->start_angle + (servo->target_angle - servo->start_angle) * progress;
    }

    uint16_t new_count;
    servo_angle_to_count(servo, servo->current_angle, &new_count);
    if (new_count != servo->current_count) {
        pca9685_set_pwm(servo->channel, 0, new_count);
        servo->current_count = new_count;
    }

    return OK;
}

ErrorCode_e servo_angle_to_count(const servo_t *servo, float angle, uint16_t *count) {
    if (!servo || !servo->initialized) return ERR_INVALID_PARAM;

    // Apply quadratic calibration: count = a*angle² + b*angle + c
    float count_float =
        servo->calib.a * angle * angle + servo->calib.b * angle + servo->calib.c;

    // Round and clamp to counter limits
    *count = MAX(MIN((uint16_t)(count_float + 0.5f), servo->max_count),
                 servo->min_count);

    return OK;
}

ErrorCode_e servo_is_moving(const servo_t *servo) {
    if (!servo || !servo->initialized) return ERR_INVALID_PARAM;

    return servo->state == SERVO_STATE_MOVING ? OK : ERR_ERROR;
}

ErrorCode_e servo_stop(servo_t *servo) {
    if (!servo || !servo->initialized) return ERR_INVALID_PARAM;

    if (servo->state == SERVO_STATE_MOVING) {
        servo->interp_start_us = 0;
        servo->state = SERVO_STATE_IDLE;
        servo->target_angle = servo->current_angle;
    }

    return OK;
}

ErrorCode_e servo_set_count_direct(servo_t *servo, uint16_t count) {
    if (!servo || !servo->initialized) return ERR_INVALID_PARAM;

    servo->current_count = count;
    return pca9685_set_pwm(servo->channel, 0, count);
}

ErrorCode_e servo_set_pulse_direct(servo_t *servo, uint16_t pulse_us) {
    if (!servo || !servo->initialized) return ERR_INVALID_PARAM;

    return pca9685_set_pulse_width(servo->channel, pulse_us);
}
