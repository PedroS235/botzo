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
#include "servo_mgr.h"

uint8_t led_state = false;
uint16_t servo_angle = 0;

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

void move_servo() {
    static int angle = 0;
    printf("Angle: %d\n", angle);
    servo_mgr_move_to(0, angle);
    angle = (angle + 45) % 270;

    // static uint16_t pulse = 350;
    // if (get_absolute_time() - last_time >= 250000) {
    //     printf("Current pulse width: %u\n", pulse);
    //     servo_mgr_move_pulse(0, pulse);
    //     pulse = (pulse + 10) % 2800;
    //     last_time = get_absolute_time();
    // }
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
    servo_mgr_init(i2c_default, PCA9685_DEFAULT_I2C_ADDRESS);

    scheduler_add_task(1000, 1, toggle_led);
    scheduler_add_task(2000, 1, move_servo);
    scheduler_add_task(10, 5, read_imu);
    scheduler_add_task(50, 2, activity_led);
    // scheduler_add_task(500, 1, status_task);

    sleep_ms(5000);
    printf("Starting auto calibration\n");

    pca9685_auto_calibrate_osc_blocking_ex(3, 0, 500000);

    while (1) {
        scheduler_run();
    }

    // NEVER REACHED
}
