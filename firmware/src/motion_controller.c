#include "motion_controller.h"

#include <string.h>

#include "drivers/pca9685/pca9685.h"
#include "error.h"
#include "servo.h"

static motion_controller_state_t mc_state = {0};

static uint8_t get_channel(leg_id_e leg, joint_id_e joint) {
    return mc_state.mapping[leg][joint].channel;
}

ErrorCode_e motion_controller_init(const motion_controller_config_t *config) {
    if (!config || !config->i2c) return ERR_INVALID_PARAM;
    if (mc_state.initialized) return ERR_DRIVER_ALREADY_INITIALIZED;

    memcpy(&mc_state.config, config, sizeof(motion_controller_config_t));
    PCA9685Config_t pca9685_config = {.freq = config->pwm_frequency};

    ErrorCode_e err = pca9685_init(config->i2c, config->pca9685_addr, &pca9685_config);
    if (err != OK) return err;

    mc_state.initialized = true;
    return OK;
}

ErrorCode_e motion_controller_load_calibrations(const servo_config_t calib[12]) {
    if (!mc_state.initialized) return ERR_MOTION_CONTROLLER_NOT_INITIALIZED;
    if (!calib) return ERR_INVALID_PARAM;

    for (uint8_t i = 0; i < 12; i++) {
        ErrorCode_e err = servo_init(&mc_state.servos[i], &calib[i]);
        if (err != OK) return err;
    }

    return OK;
}

ErrorCode_e motion_controller_set_mapping(leg_id_e leg,
                                          joint_id_e joint,
                                          uint8_t channel) {
    if (!mc_state.initialized) return ERR_MOTION_CONTROLLER_NOT_INITIALIZED;
    if (leg >= NUM_LEGS) return ERR_INVALID_LEG_ID;
    if (joint >= NUM_JOINTS_PER_LEG) return ERR_INVALID_JOINT_ID;
    if (channel >= 12) return ERR_SERVO_INVALID_CHANNEL;

    mc_state.mapping[leg][joint].channel = channel;
    return OK;
}

ErrorCode_e motion_controller_move_servo(uint8_t channel,
                                         float angle,
                                         uint32_t duration_ms) {
    if (!mc_state.initialized) return ERR_MOTION_CONTROLLER_NOT_INITIALIZED;
    if (channel >= 12) return ERR_SERVO_INVALID_CHANNEL;

    if (duration_ms == 0) duration_ms = mc_state.config.default_interp_time_ms;

    return servo_set_target(&mc_state.servos[channel], angle, duration_ms);
}

ErrorCode_e motion_controller_move_joint(leg_id_e leg,
                                         joint_id_e joint,
                                         float angle,
                                         uint32_t duration_ms) {
    if (!mc_state.initialized) return ERR_MOTION_CONTROLLER_NOT_INITIALIZED;
    if (leg >= NUM_LEGS) return ERR_INVALID_LEG_ID;
    if (joint >= NUM_JOINTS_PER_LEG) return ERR_INVALID_JOINT_ID;

    if (duration_ms == 0) duration_ms = mc_state.config.default_interp_time_ms;

    uint8_t channel = get_channel(leg, joint);
    return servo_set_target(&mc_state.servos[channel], angle, duration_ms);
}

ErrorCode_e motion_controller_move_leg(leg_id_e leg,
                                       float coxa,
                                       float femur,
                                       float tibia,
                                       uint32_t duration_ms) {
    if (!mc_state.initialized) return ERR_MOTION_CONTROLLER_NOT_INITIALIZED;
    if (leg >= NUM_LEGS) return ERR_INVALID_LEG_ID;

    if (duration_ms == 0) duration_ms = mc_state.config.default_interp_time_ms;

    uint8_t ch_coxa = get_channel(leg, JOINT_COXA);
    uint8_t ch_femur = get_channel(leg, JOINT_FEMUR);
    uint8_t ch_tibia = get_channel(leg, JOINT_TIBIA);

    ErrorCode_e err;
    err = servo_set_target(&mc_state.servos[ch_coxa], coxa, duration_ms);
    if (err != OK) return err;

    err = servo_set_target(&mc_state.servos[ch_femur], femur, duration_ms);
    if (err != OK) return err;

    err = servo_set_target(&mc_state.servos[ch_tibia], tibia, duration_ms);
    return err;
}

ErrorCode_e motion_controller_move_all(const float angles[12], uint32_t duration_ms) {
    if (!mc_state.initialized) return ERR_MOTION_CONTROLLER_NOT_INITIALIZED;
    if (!angles) return ERR_INVALID_PARAM;

    if (duration_ms == 0) duration_ms = mc_state.config.default_interp_time_ms;

    for (uint8_t i = 0; i < 12; i++) {
        ErrorCode_e err = servo_set_target(&mc_state.servos[i], angles[i], duration_ms);
        if (err != OK) return err;
    }

    return OK;
}

ErrorCode_e motion_controller_get_angle(uint8_t channel, float *angle) {
    if (!mc_state.initialized) return ERR_MOTION_CONTROLLER_NOT_INITIALIZED;
    if (channel >= 12) return ERR_SERVO_INVALID_CHANNEL;
    if (!angle) return ERR_INVALID_PARAM;

    return servo_get_angle(&mc_state.servos[channel], angle);
}

ErrorCode_e motion_controller_get_joint_angle(leg_id_e leg,
                                              joint_id_e joint,
                                              float *angle) {
    if (!mc_state.initialized) return ERR_MOTION_CONTROLLER_NOT_INITIALIZED;
    if (leg >= NUM_LEGS) return ERR_INVALID_LEG_ID;
    if (joint >= NUM_JOINTS_PER_LEG) return ERR_INVALID_JOINT_ID;
    if (!angle) return ERR_INVALID_PARAM;

    uint8_t channel = get_channel(leg, joint);
    return servo_get_angle(&mc_state.servos[channel], angle);
}

ErrorCode_e motion_controller_is_moving(bool *is_moving) {
    if (!mc_state.initialized) return ERR_MOTION_CONTROLLER_NOT_INITIALIZED;
    if (!is_moving) return ERR_INVALID_PARAM;

    *is_moving = false;
    for (uint8_t i = 0; i < 12; i++) {
        if (servo_is_moving(&mc_state.servos[i]) == OK) {
            *is_moving = true;
            return OK;
        }
    }

    return OK;
}

ErrorCode_e motion_controller_stop_all(void) {
    if (!mc_state.initialized) return ERR_MOTION_CONTROLLER_NOT_INITIALIZED;

    for (uint8_t i = 0; i < 12; i++) {
        ErrorCode_e err = servo_stop(&mc_state.servos[i]);
        if (err != OK) return err;
    }

    return OK;
}

void motion_controller_update_task(void) {
    if (!mc_state.initialized) return;

    for (uint8_t i = 0; i < 12; i++) {
        servo_update(&mc_state.servos[i]);
    }
}

ErrorCode_e motion_controller_set_count_direct(uint8_t channel, uint16_t count) {
    if (!mc_state.initialized) return ERR_MOTION_CONTROLLER_NOT_INITIALIZED;
    if (channel >= 12) return ERR_SERVO_INVALID_CHANNEL;

    return servo_set_count_direct(&mc_state.servos[channel], count);
}

ErrorCode_e motion_controller_set_pulse_direct(uint8_t channel, uint16_t pulse_us) {
    if (!mc_state.initialized) return ERR_MOTION_CONTROLLER_NOT_INITIALIZED;
    if (channel >= 12) return ERR_SERVO_INVALID_CHANNEL;

    return servo_set_pulse_direct(&mc_state.servos[channel], pulse_us);
}
