#ifndef DRIVERS_PCA9685_H
#define DRIVERS_PCA9685_H

#include "error.h"
#include <hardware/i2c.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint8_t address;
} PCA9685Config_s;

ErrorCode_e pca9685_init(i2c_inst_t *i2c, const PCA9685Config_s *config);
void pca9685_deinit(void);
ErrorCode_e pca9685_is_connected(void);
ErrorCode_e pca9685_sleep(void);
ErrorCode_e pca9685_wakeup(void);
ErrorCode_e pca9685_reset(void);

ErrorCode_e pca9685_get_prescale(uint8_t *prescale);
ErrorCode_e pca9685_set_prescale(uint8_t prescale);
ErrorCode_e pca9685_get_frequency(uint8_t *prescale);
ErrorCode_e pca9685_set_frequency(float freq);
ErrorCode_e pca9685_set_ai();

ErrorCode_e pca9685_set_pwm(uint8_t idx, uint16_t on, uint16_t off);
ErrorCode_e pca9685_set_pin(uint8_t idx);

ErrorCode_e read_register(i2c_inst_t *bus, uint8_t addr, uint8_t reg,
                          uint8_t *dest);
ErrorCode_e write_register(i2c_inst_t *bus, uint8_t addr, uint8_t reg,
                           uint8_t data);
#endif // !DRIVERS_PCA9685_H
