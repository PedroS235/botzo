#include "drivers/pca9685/pca9685.h"
#include "drivers/pca9685/registers.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include <pico/time.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

#define SERVOMIN 150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN                                                                  \
  600 // This is the rounded 'minimum' microsecond length based on the minimum
      // pulse of 150
#define USMAX                                                                  \
  2400 // This is the rounded 'maximum' microsecond length based on the maximum
       // pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

int main() {
  // Enable UART so we can print status output
  stdio_init_all();
  gpio_init(25);
  gpio_set_dir(25, GPIO_OUT);
  i2c_init(i2c_default, 100 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
  // Make the I2C pins available to picotool
  bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN,
                             GPIO_FUNC_I2C));

  uint8_t ret;
  PCA9685Config_s config = {.address = 0x40};
  sleep_ms(2000);
  pca9685_init(i2c_default, &config);
  if (pca9685_reset() < 0) {
    printf("Problem occurend when reset called\n");
  }

  pca9685_set_frequency(SERVO_FREQ);
  uint8_t data;

  while (1) {
    read_register(i2c_default, 0x40, PCA9685_MODE1, &data);
    printf("MODE 1 Register: %x\n", data);
    pca9685_set_pwm(0, 4096, 0);

    uint8_t prescale;
    pca9685_get_prescale(&prescale);
    printf("PRESCALE: %d\n", prescale);

    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
      pca9685_set_pwm(0, 0, pulselen);
    }

    sleep_ms(500);
    for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
      pca9685_set_pwm(0, 0, pulselen);
    }

    gpio_put(25, 0);
    sleep_ms(250);
    gpio_put(25, 1);
    sleep_ms(250);
  }
}
