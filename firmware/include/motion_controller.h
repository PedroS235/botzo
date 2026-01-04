#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

// Leg enumeration
#include <hardware/i2c.h>
#include <stdint.h>

#include "servo.h"
typedef enum {
    LEG_FRONT_LEFT = 0,
    LEG_FRONT_RIGHT = 1,
    LEG_BACK_LEFT = 2,
    LEG_BACK_RIGHT = 3,
    NUM_LEGS = 4
} leg_id_e;

// Joint enumeration
typedef enum {
    JOINT_COXA = 0,   // Hip rotation
    JOINT_FEMUR = 1,  // Upper leg
    JOINT_TIBIA = 2,  // Lower leg
    NUM_JOINTS_PER_LEG = 3
} joint_id_e;

// Channel mapping structure
typedef struct {
    uint8_t channel;  // PCA9685 channel (0-15)
} channel_mapping_t;

// Configuration
typedef struct {
    i2c_inst_t *i2c;
    uint8_t pca9685_addr;
    uint16_t pwm_frequency;           // 50Hz
    uint32_t default_interp_time_ms;  // Default transition duration (e.g., 300ms)
} motion_controller_config_t;

// Internal state
typedef struct {
    servo_t servos[12];                                       // All 12 servos
    channel_mapping_t mapping[NUM_LEGS][NUM_JOINTS_PER_LEG];  // Channel map
    motion_controller_config_t config;
    bool initialized;
} motion_controller_state_t;

// Initialization
ErrorCode_e motion_controller_init(const motion_controller_config_t *config);
ErrorCode_e motion_controller_load_calibrations(const servo_config_t calib[12]);
ErrorCode_e motion_controller_set_mapping(leg_id_e leg,
                                          joint_id_e joint,
                                          uint8_t channel);

// Control methods
ErrorCode_e motion_controller_move_servo(uint8_t channel,
                                         float angle,
                                         uint32_t duration_ms);
ErrorCode_e motion_controller_move_joint(leg_id_e leg,
                                         joint_id_e joint,
                                         float angle,
                                         uint32_t duration_ms);
ErrorCode_e motion_controller_move_leg(leg_id_e leg,
                                       float coxa,
                                       float femur,
                                       float tibia,
                                       uint32_t duration_ms);
ErrorCode_e motion_controller_move_all(const float angles[12], uint32_t duration_ms);

// State queries
ErrorCode_e motion_controller_get_angle(uint8_t channel, float *angle);
ErrorCode_e motion_controller_get_joint_angle(leg_id_e leg,
                                              joint_id_e joint,
                                              float *angle);
ErrorCode_e motion_controller_is_moving(bool *is_moving);
ErrorCode_e motion_controller_stop_all(void);

// Scheduler integration
void motion_controller_update_task(void);  // Call at 10ms (100Hz)

// Testing/debug
ErrorCode_e motion_controller_set_count_direct(uint8_t channel, uint16_t count);
ErrorCode_e motion_controller_set_pulse_direct(uint8_t channel, uint16_t pulse_us);

#endif  // !MOTION_CONTROLLER_H
