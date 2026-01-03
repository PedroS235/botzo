#ifndef DRIVERS_I2C_H
#define DRIVERS_I2C_H

#include <pico/error.h>
#include <stdint.h>
#include <string.h>

#include "error.h"
#include "hardware/i2c.h"

ErrorCode_e read_register(i2c_inst_t *i2c,
                          const uint8_t addr,
                          const uint8_t reg,
                          uint8_t *dest,
                          const size_t len);

ErrorCode_e write_register(i2c_inst_t *i2c,
                           const uint8_t addr,
                           const uint8_t reg,
                           const uint8_t *data,
                           const size_t len);

#endif  // !DRIVERS_I2C_H
