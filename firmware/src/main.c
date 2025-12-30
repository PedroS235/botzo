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

uint8_t led_state = false;
servo_t test_servo;

typedef struct {
    uint32_t duration_ms;
    uint8_t state;
} led_pattern_step_t;

led_pattern_step_t pattern[] = {
    {100000, 1},  // ON 100ms
    {100000, 0},  // OFF 100ms
    {100000, 1},  // ON 100ms
    {100000, 0},  // OFF 100ms
    {1200000, 0}  // OFF 1200ms (pause)
};

void activity_led(void) {
    static uint8_t step = 0;
    static uint64_t step_start = 0;

    uint64_t now = get_absolute_time();

    if (now - step_start >= pattern[step].duration_ms) {
        step = (step + 1) % 5;  // Loop pattern
        gpio_put(25, pattern[step].state);
        step_start = now;
    }
}

void toggle_led() {
    gpio_put(26, led_state);
    led_state = !led_state;
}

volatile absolute_time_t last_time = 0;

void servo_update_task(void) { servo_update(&test_servo); }

void test_servo_movement(void) {
    static uint64_t start_time = 0;

    if (start_time == 0) {
        start_time = time_us_64();
    }

    // Calculate elapsed time in seconds
    float elapsed_sec = (time_us_64() - start_time) / 1000000.0f;

    // Sine wave: period = 4 seconds, oscillates between 0° and 180°
    float angle = 90.0f + 90.0f * sinf(2.0f * 3.14159f * elapsed_sec / 10.0f);

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

int main() {
    // Enable UART so we can print status output
    stdio_init_all();
    gpio_init(26);
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_set_dir(26, GPIO_OUT);
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(
        PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    mpu60x0_init(i2c_default, MPU60X0_DEFAULT_I2C_ADDRESS, &MPU60X0_DEFAULT_CONFIG);

    // Initialize PCA9685
    PCA9685Config_t pca_config = {.freq = 50};
    pca9685_init(i2c_default, PCA9685_DEFAULT_I2C_ADDRESS, &pca_config);

    printf("Starting PCA9685 auto calibration\n");
    pca9685_auto_calibrate_osc_blocking_ex(3, 0, 500000);

    // Initialize test servo on channel 0
    // Calibration converted from μs to counter units (at 50Hz ≈ 4.88μs/count)
    // Original μs calibration: a=0.0, b=7.378, c=616.0
    // Counter calibration: divide by ~4.88
    servo_config_t servo_config = {.channel = 0,
                                   .calib = {.a = 0.00053f, .b = 1.686f, .c = 72.0f},
                                   .min_angle = 0.0f,
                                   .max_angle = 270.0f,
                                   .min_count = 72,    // 500μs ≈ 102 counts
                                   .max_count = 553};  // 2500μs ≈ 512 counts
    servo_init(&test_servo, &servo_config);

    printf("\n=== Servo Test Initialized ===\n");
    printf("Channel: 0\n");
    printf("Calibration (counter): a=%.3f, b=%.3f, c=%.1f\n",
           servo_config.calib.a,
           servo_config.calib.b,
           servo_config.calib.c);
    printf("Range: %.0f° - %.0f° (counts: %u - %u)\n",
           servo_config.min_angle,
           servo_config.max_angle,
           servo_config.min_count,
           servo_config.max_count);
    printf("PCA9685 precision: %.4f μs/count\n\n", test_servo.us_per_count);

    // scheduler_add_task(1000, 1, toggle_led);
    scheduler_add_task(100, 3, test_servo_movement);  // Update sine wave every 100ms
    scheduler_add_task(10, 5, servo_update_task);     // Update servo at 10ms (100Hz)
    scheduler_add_task(10, 5, read_imu);
    scheduler_add_task(50, 2, activity_led);
    scheduler_add_task(500, 1, status_task);

    while (1) {
        scheduler_run();
    }

    // NEVER REACHED
}
