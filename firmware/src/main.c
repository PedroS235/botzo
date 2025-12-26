#include <pico/time.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

#include "drivers/mpu60x0/mpu60x0.h"
#include "drivers/pca9685/pca9685.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "scheduler.h"

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
    printf("Toggle led\n");
}

void move_servo() {
    if (!led_state)
        pca9685_set_pwm(1, 0, 4096);
    else
        pca9685_set_pwm(1, 4096, 0);
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

    uint8_t ret;
    PCA9685Config_s config = {.freq = 50};
    pca9685_init(i2c_default, PCA9685_DEFAULT_I2C_ADDRESS, &config);
    mpu60x0_init(i2c_default, MPU60X0_DEFAULT_I2C_ADDRESS, &MPU60X0_DEFAULT_CONFIG);

    scheduler_add_task(1000, 1, toggle_led);
    scheduler_add_task(1000, 1, move_servo);
    scheduler_add_task(10, 5, read_imu);
    scheduler_add_task(50, 2, activity_led);
    scheduler_add_task(500, 1, status_task);

    while (1) {
        scheduler_run();
    }

    // NEVER REACHED
}
