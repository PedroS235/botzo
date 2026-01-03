#include <math.h>
#include <pico/time.h>
#include <pico/types.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

#include "drivers/mpu60x0/mpu60x0.h"
#include "drivers/pca9685/pca9685.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "scheduler.h"
#include "servo.h"

#define ACTIVITY_LED 25

uint8_t led_state = false;
servo_t test_servo = {0};

void toggle_led() {
    gpio_put(ACTIVITY_LED, led_state);
    led_state = !led_state;
}

void servo_update_task(void) { servo_update(&test_servo); }

void test_servo_movement(void) {
    static uint64_t start_time = 0;

    if (start_time == 0) {
        start_time = time_us_64();
    }

    // Calculate elapsed time in seconds
    float elapsed_sec = (time_us_64() - start_time) / 1000000.0f;

    // Sine wave: period = 4 seconds, oscillates between 0° and 180°
    float angle = 90.0f + 90.0f * sinf(2.0f * 3.14159f * elapsed_sec / 4.0f);

    float current_angle;
    servo_get_angle(&test_servo, &current_angle);

    printf("t=%.2fs | Target: %.1f° | Current: %.1f°\n",
           elapsed_sec,
           angle,
           current_angle);

    servo_set_target(&test_servo, angle, 150);  // 150ms smooth transition
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

    // Init PCA9685
    PCA9685Config_t pca_config = {.freq = 50};
    err = pca9685_init(i2c_default, PCA9685_DEFAULT_I2C_ADDRESS, &pca_config);
    if (err != OK) {
        printf("Failed to initialize PCA9685 with err: %d", err);
    }

    printf("Starting PCA9685 auto calibration\n");
    err = pca9685_auto_calibrate_osc_blocking_ex(3, 15, 500000);
    if (err != OK) {
        printf("Failed to calibrate PCA9685 int. osc. with err: %d", err);
    }

    // Init Servo test
    servo_config_t servo_config = {.channel = 0,
                                   .calib = {.a = 0.0f, .b = 1.0f, .c = 72.0f},
                                   .min_angle = 0.0f,
                                   .max_angle = 270.0f,
                                   .min_count = 72,    // 500μs ≈ 102 counts
                                   .max_count = 553};  // 2500μs ≈ 512 counts
    err = servo_init(&test_servo, &servo_config);
    if (err != OK) {
        printf("Failed to initialize servo with err: %d", err);
    }

    scheduler_add_task(1000, 3, toggle_led);
    scheduler_add_task(100, 2, test_servo_movement);  // Update sine wave every 100ms
    scheduler_add_task(10, 1, servo_update_task);     // Update servo at 10ms (100Hz)
    scheduler_add_task(10, 1, read_imu);
    // scheduler_add_task(500, 1, status_task);

    while (1) {
        scheduler_run();
    }

    // NEVER REACHED
}
