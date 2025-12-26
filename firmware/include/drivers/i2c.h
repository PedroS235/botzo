#ifndef DRIVERS_I2C_H
#define DRIVERS_I2C_H

#include <pico/error.h>
#include <stdint.h>
#include <string.h>

#include "error.h"
#include "hardware/i2c.h"

inline ErrorCode_e read_register(i2c_inst_t *i2c,
                                 const uint8_t addr,
                                 const uint8_t reg,
                                 uint8_t *dest,
                                 const size_t len) {
    assert(dest != NULL);
    assert(len > 0U);
    assert(len <= PCA9685_MAX_DATA_BYTES);

    if (dest == NULL) {
        return ERR_INVALID_PARAM;
    }
    if (len == 0U) {
        return ERR_INVALID_PARAM;
    }

    int ret = i2c_write_blocking(i2c, addr, &reg, 1, true);
    if ((ret == PICO_ERROR_GENERIC) || (ret != 1)) {
        return ERR_I2C_ERROR;
    }

    ret = i2c_read_blocking(i2c, addr, dest, len, false);
    if ((ret == PICO_ERROR_GENERIC) || (ret != (int)len)) {
        return ERR_I2C_ERROR;
    }

    return OK;
}

inline ErrorCode_e write_register(i2c_inst_t *i2c,
                                  const uint8_t addr,
                                  const uint8_t reg,
                                  const uint8_t *data,
                                  const size_t len) {
    assert(data != NULL);
    assert(len > 0U);
    assert(len <= PCA9685_MAX_DATA_BYTES);

    if (data == NULL) {
        return ERR_INVALID_PARAM;
    }
    if (len == 0U) {
        return ERR_INVALID_PARAM;
    }

    // TODO: Change size to a fixed max?
    uint8_t buf[len + 1];
    buf[0] = reg;
    (void)memcpy(&buf[1], data, len);

    int ret = i2c_write_blocking(i2c, addr, buf, len + 1U, false);
    if ((ret == PICO_ERROR_GENERIC) || (ret != (int)(len + 1U))) {
        return ERR_I2C_ERROR;
    }

    return OK;
}

#endif  // !DRIVERS_I2C_H
