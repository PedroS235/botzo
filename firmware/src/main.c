#include <math.h>
#include <pico/time.h>
#include <pico/types.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

#include "drivers/mpu60x0/mpu60x0.h"
#include "drivers/pca9685/pca9685.h"
#include "hardware/i2c.h"
#include "motion_controller.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "scheduler.h"
#include "servo.h"

#define ACTIVITY_LED 25

uint8_t led_state = false;

void toggle_led() {
    gpio_put(ACTIVITY_LED, led_state);
    led_state = !led_state;
}

void test_motion_controller(void) {
    static uint8_t step = 0;
    static uint64_t last_command_time = 0;
    static bool waiting = false;

    bool is_moving = false;
    motion_controller_is_moving(&is_moving);

    if (waiting && is_moving) {
        return;
    }

    if (waiting && !is_moving) {
        uint64_t now = time_us_64();
        if ((now - last_command_time) < 3000000) {
            return;
        }
        waiting = false;
    }

    switch (step) {
        case 0:
            printf("Position 1: All servos to center (90°)\n");
            motion_controller_move_leg(LEG_FRONT_LEFT, 90, 90, 90, 500);
            waiting = true;
            last_command_time = time_us_64();
            break;
        case 1:
            printf("Position 2: Coxa=45°, Femur=60°, Tibia=120°\n");
            motion_controller_move_leg(LEG_FRONT_LEFT, 45, 60, 120, 500);
            waiting = true;
            last_command_time = time_us_64();
            break;
        case 2:
            printf("Position 3: Coxa=135°, Femur=120°, Tibia=60°\n");
            motion_controller_move_leg(LEG_FRONT_LEFT, 135, 120, 60, 500);
            waiting = true;
            last_command_time = time_us_64();
            break;
        case 3:
            printf("Position 4: All servos to 0°\n");
            motion_controller_move_leg(LEG_FRONT_LEFT, 0, 0, 0, 500);
            waiting = true;
            last_command_time = time_us_64();
            break;
        case 4:
            printf("Position 5: All servos to 180°\n");
            motion_controller_move_leg(LEG_FRONT_LEFT, 180, 180, 180, 500);
            waiting = true;
            last_command_time = time_us_64();
            break;
    }

    step = (step + 1) % 5;
}

float temp;
Vec3_t accel;
Vec3_t gyro;

void read_imu() {
    mpu60x0_read_accel(&accel);
    mpu60x0_read_gyro(&gyro);
    mpu60x0_read_temp(&temp);
    //
    // printf("ACCEL: %d, %d, %d\n", accel.x, accel.y, accel.z);
    // printf("GYRO: %d, %d, %d\n", gyro.x, gyro.y, gyro.z);
    // printf("TEMP: %f\n", temp);
}

void status_task(void) {
    scheduler_print_stats();
    scheduler_reset_stats();
}

void setup_i2c() {
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
}

void setup_activity_led() {
    gpio_init(ACTIVITY_LED);
    gpio_set_dir(ACTIVITY_LED, GPIO_OUT);
}

int main() {
    ErrorCode_e err;
    sleep_ms(2000);

    stdio_init_all();
    setup_i2c();
    setup_activity_led();

    // Init IMU
    err =
        mpu60x0_init(i2c_default, MPU60X0_DEFAULT_I2C_ADDRESS, &MPU60X0_DEFAULT_CONFIG);
    if (err != OK) {
        printf("Failed to initialize MPU60X0 (IMU) with err: %d", err);
    }

    // Init Motion Controller
    motion_controller_config_t mc_config = {.i2c = i2c_default,
                                            .pca9685_addr = PCA9685_DEFAULT_I2C_ADDRESS,
                                            .pwm_frequency = 50,
                                            .default_interp_time_ms = 300};
    err = motion_controller_init(&mc_config);
    if (err != OK) {
        printf("Failed to initialize motion controller with err: %d\n", err);
    }

    printf("Starting PCA9685 auto calibration\n");
    // err = pca9685_auto_calibrate_osc_blocking_ex(3, 15, 500000);
    // if (err != OK) {
    //     printf("Failed to calibrate PCA9685 int. osc. with err: %d\n", err);
    // }

    // Define calibrations for all 12 servos (all use 0-180° range)
    static const servo_config_t default_calibrations[12] = {
        [0] = {.channel = 0,
               .calib = {.a = 0.0f, .b = 1.781f, .c = 72.0f},
               .min_angle = 0.0f,
               .max_angle = 180.0f,
               .min_count = 72,
               .max_count = 553},
        [1] = {.channel = 1,
               .calib = {.a = 0.0f, .b = 1.781f, .c = 72.0f},
               .min_angle = 0.0f,
               .max_angle = 180.0f,
               .min_count = 72,
               .max_count = 553},
        [2] = {.channel = 2,
               .calib = {.a = 0.0f, .b = 1.781f, .c = 72.0f},
               .min_angle = 0.0f,
               .max_angle = 180.0f,
               .min_count = 72,
               .max_count = 553},
        [3] = {.channel = 3,
               .calib = {.a = 0.0f, .b = 1.781f, .c = 72.0f},
               .min_angle = 0.0f,
               .max_angle = 180.0f,
               .min_count = 72,
               .max_count = 553},
        [4] = {.channel = 4,
               .calib = {.a = 0.0f, .b = 1.781f, .c = 72.0f},
               .min_angle = 0.0f,
               .max_angle = 180.0f,
               .min_count = 72,
               .max_count = 553},
        [5] = {.channel = 5,
               .calib = {.a = 0.0f, .b = 1.781f, .c = 72.0f},
               .min_angle = 0.0f,
               .max_angle = 180.0f,
               .min_count = 72,
               .max_count = 553},
        [6] = {.channel = 6,
               .calib = {.a = 0.0f, .b = 1.781f, .c = 72.0f},
               .min_angle = 0.0f,
               .max_angle = 180.0f,
               .min_count = 72,
               .max_count = 553},
        [7] = {.channel = 7,
               .calib = {.a = 0.0f, .b = 1.781f, .c = 72.0f},
               .min_angle = 0.0f,
               .max_angle = 180.0f,
               .min_count = 72,
               .max_count = 553},
        [8] = {.channel = 8,
               .calib = {.a = 0.0f, .b = 1.781f, .c = 72.0f},
               .min_angle = 0.0f,
               .max_angle = 180.0f,
               .min_count = 72,
               .max_count = 553},
        [9] = {.channel = 9,
               .calib = {.a = 0.0f, .b = 1.781f, .c = 72.0f},
               .min_angle = 0.0f,
               .max_angle = 180.0f,
               .min_count = 72,
               .max_count = 553},
        [10] = {.channel = 10,
                .calib = {.a = 0.0f, .b = 1.781f, .c = 72.0f},
                .min_angle = 0.0f,
                .max_angle = 180.0f,
                .min_count = 72,
                .max_count = 553},
        [11] = {.channel = 11,
                .calib = {.a = 0.0f, .b = 1.781f, .c = 72.0f},
                .min_angle = 0.0f,
                .max_angle = 180.0f,
                .min_count = 72,
                .max_count = 553}};

    err = motion_controller_load_calibrations(default_calibrations);
    if (err != OK) {
        printf("Failed to load calibrations with err: %d\n", err);
    }

    // Setup channel mapping (Front-Left: 0-2, Front-Right: 3-5, Back-Left: 6-8,
    // Back-Right: 9-11)
    motion_controller_set_mapping(LEG_FRONT_LEFT, JOINT_COXA, 0);
    motion_controller_set_mapping(LEG_FRONT_LEFT, JOINT_FEMUR, 1);
    motion_controller_set_mapping(LEG_FRONT_LEFT, JOINT_TIBIA, 2);
    motion_controller_set_mapping(LEG_FRONT_RIGHT, JOINT_COXA, 3);
    motion_controller_set_mapping(LEG_FRONT_RIGHT, JOINT_FEMUR, 4);
    motion_controller_set_mapping(LEG_FRONT_RIGHT, JOINT_TIBIA, 5);
    motion_controller_set_mapping(LEG_BACK_LEFT, JOINT_COXA, 6);
    motion_controller_set_mapping(LEG_BACK_LEFT, JOINT_FEMUR, 7);
    motion_controller_set_mapping(LEG_BACK_LEFT, JOINT_TIBIA, 8);
    motion_controller_set_mapping(LEG_BACK_RIGHT, JOINT_COXA, 9);
    motion_controller_set_mapping(LEG_BACK_RIGHT, JOINT_FEMUR, 10);
    motion_controller_set_mapping(LEG_BACK_RIGHT, JOINT_TIBIA, 11);

    printf("Motion controller initialized\n");

    scheduler_add_task(1000, 3, toggle_led);
    scheduler_add_task(100, 2, test_motion_controller);
    scheduler_add_task(
        10, 1, motion_controller_update_task);  // Update servos at 10ms (100Hz)
    scheduler_add_task(10, 1, read_imu);
    // scheduler_add_task(500, 1, status_task);

    while (1) {
        scheduler_run();
    }

    // NEVER REACHED
}
