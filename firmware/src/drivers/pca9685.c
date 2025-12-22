#include "drivers/pca9685/pca9685.h"
#include "drivers/pca9685/registers.h"
#include "error.h"
#include "hardware/i2c.h"
#include <pico/error.h>
#include <pico/time.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/_intsup.h>
#include <sys/types.h>

#define PCA9685_OSCCLOCK 25000000

typedef struct {
  i2c_inst_t *i2c;
  const PCA9685Config_s *config;
  long oscclock;
} PCA9685_t;

static PCA9685_t *device;

ErrorCode_e read_register(i2c_inst_t *bus, uint8_t addr, uint8_t reg,
                          uint8_t *dest) {
  int ret;

  // Write target register to control register
  ret = i2c_write_blocking(bus, addr, &reg, 1, true);
  if (ret == PICO_ERROR_GENERIC || ret != 1) {
    return ERR_I2C_ERROR;
  }

  // Read value from target register
  ret = i2c_read_blocking(bus, addr, dest, 1, false);
  if (ret == PICO_ERROR_GENERIC || ret != 1) {
    return ERR_I2C_ERROR;
  }

  return OK;
}

ErrorCode_e write_register(i2c_inst_t *bus, uint8_t addr, uint8_t reg,
                           uint8_t data) {
  uint8_t buf[2];
  buf[0] = reg;
  memcpy(buf + 1, &data, 1);

  int ret = i2c_write_blocking(bus, addr, buf, 2, false);
  if (ret == PICO_ERROR_GENERIC || ret != 2) {
    return ERR_I2C_ERROR;
  }

  return OK;
}

ErrorCode_e pca9685_init(i2c_inst_t *i2c, const PCA9685Config_s *config) {
  // Driver is already initialized
  if (device != NULL)
    return OK;

  device = (PCA9685_t *)malloc(sizeof(PCA9685_t));
  if (device == NULL) {
    return ERR_NO_MEMORY;
  }

  device->config = config;
  device->i2c = i2c;
  device->oscclock = 25000000;

  return OK;
}

void pca9685_deinit(void) {
  if (device != NULL) {
    free(device);
    device = NULL;
  }
}

ErrorCode_e pca9685_is_connected(void) {
  if (device == NULL)
    return ERR_DRIVER_NOT_INITIALIZED;
  uint8_t rxdata;

  int ret = read_register(device->i2c, device->config->address, PCA9685_MODE1,
                          &rxdata);
  if (ret < 0 || rxdata != DEFAULT_MODE1)
    return ret;

  return OK;
}

ErrorCode_e pca9685_sleep(void) {
  uint8_t awake;
  int ret = read_register(device->i2c, device->config->address, PCA9685_MODE1,
                          &awake);
  uint8_t sleep = awake | MODE1_SLEEP; // set sleep bit high
  write_register(device->i2c, device->config->address, PCA9685_MODE1, sleep);
  sleep_ms(5);
  return OK;
}
ErrorCode_e pca9685_wakeup(void) {
  uint8_t sleep;
  read_register(device->i2c, device->config->address, PCA9685_MODE1, &sleep);
  uint8_t wakeup = sleep & ~MODE1_SLEEP; // set sleep bit low
  write_register(device->i2c, device->config->address, PCA9685_MODE1, wakeup);

  return OK;
}

ErrorCode_e pca9685_get_prescale(uint8_t *value) {
  return read_register(device->i2c, device->config->address, PCA9685_PRESCALE,
                       value);
}

ErrorCode_e pca9685_set_prescale(uint8_t prescale) {
  pca9685_sleep();
  return write_register(device->i2c, device->config->address, PCA9685_PRESCALE,
                        prescale);
}

ErrorCode_e pca9685_set_frequency(float freq) {
  float prescaleval = ((25000000 / (freq * 4096.0)) + 0.5) - 1;
  if (prescaleval < PCA9685_PRESCALE_MIN)
    prescaleval = PCA9685_PRESCALE_MIN;
  if (prescaleval > PCA9685_PRESCALE_MAX)
    prescaleval = PCA9685_PRESCALE_MAX;
  uint8_t prescale = (uint8_t)prescaleval;

  uint8_t oldmode;
  read_register(device->i2c, device->config->address, PCA9685_MODE1, &oldmode);

  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  pca9685_sleep();
  write_register(device->i2c, device->config->address, PCA9685_PRESCALE,
                 prescale);
  write_register(device->i2c, device->config->address, PCA9685_MODE1, oldmode);
  sleep_ms(5);
  // This sets the MODE1 register to turn on auto increment.
  write_register(device->i2c, device->config->address, PCA9685_MODE1,
                 oldmode | MODE1_RESTART | MODE1_AI);

  return OK;
}

ErrorCode_e pca9685_reset(void) {
  int ret = write_register(device->i2c, device->config->address, PCA9685_MODE1,
                           MODE1_RESTART);
  if (ret < 0) {
    return ret;
  }
  sleep_ms(10);

  return OK;
}

ErrorCode_e pca9685_set_ai() {
  write_register(device->i2c, device->config->address, PCA9685_MODE1, MODE1_AI);
  return OK;
}

ErrorCode_e pca9685_set_pwm(uint8_t idx, uint16_t on, uint16_t off) {
  uint8_t buffer[5];
  buffer[0] = PCA9685_LED0_ON_L + 4 * idx;
  buffer[1] = on;
  buffer[2] = on >> 8;
  buffer[3] = off;
  buffer[4] = off >> 8;
  int ret = i2c_write_blocking(device->i2c, device->config->address, buffer, 5,
                               false);

  return ret;
}
