#include "drivers/pca9685/pca9685.h"

#include <assert.h>
#include <hardware/gpio.h>
#include <pico/error.h>
#include <pico/time.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#include "drivers/i2c.h"
#include "drivers/pca9685/registers.h"
#include "error.h"
#include "hardware/i2c.h"

#define PCA9685_OSCCLOCK 25000000
#define PCA9685_MAX_CHANNELS 16U
#define PCA9685_MAX_PWM_VALUE 4096U
#define PCA9685_MAX_DATA_BYTES 4U
#define PCA9685_MAX_I2C_WRITE_SIZE (PCA9685_MAX_DATA_BYTES + 1U)
#define PCA9685_RESET_DELAY_MS 10U
#define PCA9685_SLEEP_DELAY_MS 5U
#define PCA9685_PRESCALE_MIN 3U
#define PCA9685_PRESCALE_MAX 255U

#define DEFAULT_MODE1 0x11U

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    long oscclock;
    uint8_t mode1_reg_val;
    const PCA9685Config_s *config;
    bool initialized;
} PCA9685_t;

static PCA9685_t driver = {0};

static ErrorCode_e read_register8(uint8_t reg, uint8_t *dest) {
    assert(dest != NULL);

    if (dest == NULL) {
        return ERR_INVALID_PARAM;
    }

    return read_register(driver.i2c, driver.addr, reg, dest, 1U);
}

static ErrorCode_e write_register8(uint8_t reg, uint8_t data) {
    return write_register(driver.i2c, driver.addr, reg, &data, 1U);
}

ErrorCode_e pca9685_init(i2c_inst_t *i2c, uint8_t addr, const PCA9685Config_s *config) {
    if (i2c == NULL) return ERR_INVALID_PARAM;
    if (config == NULL) return ERR_INVALID_PARAM;
    if (driver.initialized) return ERR_DRIVER_ALREADY_INITIALIZED;

    driver.config = config;
    driver.addr = addr;
    driver.i2c = i2c;
    driver.oscclock = PCA9685_OSCCLOCK;
    driver.mode1_reg_val = DEFAULT_MODE1;
    driver.initialized = true;

    ErrorCode_e err = pca9685_is_connected();
    if (err != OK) {
        driver.initialized = false;
        return ERR_SENSOR_NOT_CONNECTED;
    }

    err = pca9685_reset();
    if (err != OK) {
        driver.initialized = false;
        return err;
    }

    err = pca9685_set_frequency(driver.config->freq);
    if (err != OK) {
        driver.initialized = false;
        return err;
    }

    err = pca9685_set_auto_increment(true);
    if (err != OK) {
        driver.initialized = false;
        return err;
    }

    return OK;
}

void pca9685_deinit(void) {
    if (driver.initialized) {
        (void)memset(&driver, 0, sizeof(PCA9685_t));
    }
}

ErrorCode_e pca9685_is_connected(void) {
    if (!driver.initialized) {
        return ERR_DRIVER_NOT_INITIALIZED;
    }

    uint8_t buf = 0U;
    ErrorCode_e err = read_register8(PCA9685_MODE1, &buf);
    if (err != OK) {
        return err;
    }

    if (buf != driver.mode1_reg_val) {
        return ERR_SENSOR_NOT_CONNECTED;
    }

    return OK;
}

ErrorCode_e pca9685_sleep(void) {
    if (!driver.initialized) {
        return ERR_DRIVER_NOT_INITIALIZED;
    }

    uint8_t awake = 0U;
    ErrorCode_e err = read_register8(PCA9685_MODE1, &awake);
    if (err != OK) {
        return err;
    }

    uint8_t sleep = awake | MODE1_SLEEP;
    err = write_register8(PCA9685_MODE1, sleep);
    if (err != OK) {
        return err;
    }

    driver.mode1_reg_val = sleep;

    sleep_ms(PCA9685_SLEEP_DELAY_MS);
    return OK;
}

ErrorCode_e pca9685_wakeup(void) {
    if (!driver.initialized) {
        return ERR_DRIVER_NOT_INITIALIZED;
    }

    uint8_t sleep_mode = 0U;
    ErrorCode_e err = read_register8(PCA9685_MODE1, &sleep_mode);
    if (err != OK) {
        return err;
    }

    uint8_t wakeup = sleep_mode & ~MODE1_SLEEP;
    err = write_register8(PCA9685_MODE1, wakeup);
    if (err != OK) {
        return err;
    }

    driver.mode1_reg_val = wakeup;

    return OK;
}

ErrorCode_e pca9685_get_prescale(uint8_t *value) {
    assert(value != NULL);

    if (!driver.initialized) {
        return ERR_DRIVER_NOT_INITIALIZED;
    }
    if (value == NULL) {
        return ERR_INVALID_PARAM;
    }

    return read_register8(PCA9685_PRESCALE, value);
}

ErrorCode_e pca9685_set_prescale(uint8_t prescale) {
    if (!driver.initialized) {
        return ERR_DRIVER_NOT_INITIALIZED;
    }
    if ((prescale < PCA9685_PRESCALE_MIN) || (prescale > PCA9685_PRESCALE_MAX)) {
        return ERR_INVALID_PARAM;
    }

    ErrorCode_e err = pca9685_sleep();
    if (err != OK) {
        return err;
    }

    return write_register8(PCA9685_PRESCALE, prescale);
}

ErrorCode_e pca9685_get_frequency(uint8_t *freq) {
    assert(freq != NULL);
    *freq = driver.config->freq;
    return OK;
}

ErrorCode_e pca9685_set_frequency(float freq) {
    assert(freq > 0.0F);

    if (!driver.initialized) {
        return ERR_DRIVER_NOT_INITIALIZED;
    }
    if (freq <= 0.0F) {
        return ERR_INVALID_PARAM;
    }

    float prescaleval = ((25000000.0F / (freq * 4096.0F)) + 0.5F) - 1.0F;
    if (prescaleval < (float)PCA9685_PRESCALE_MIN) {
        prescaleval = (float)PCA9685_PRESCALE_MIN;
    }
    if (prescaleval > (float)PCA9685_PRESCALE_MAX) {
        prescaleval = (float)PCA9685_PRESCALE_MAX;
    }
    uint8_t prescale = (uint8_t)prescaleval;

    uint8_t oldmode = 0U;
    ErrorCode_e err = read_register8(PCA9685_MODE1, &oldmode);
    if (err != OK) {
        return err;
    }

    err = pca9685_set_prescale(prescale);
    if (err != OK) {
        return err;
    }

    err = write_register8(PCA9685_MODE1, oldmode | MODE1_RESTART);
    if (err != OK) {
        return err;
    }

    return OK;
}

ErrorCode_e pca9685_reset(void) {
    if (!driver.initialized) {
        return ERR_DRIVER_NOT_INITIALIZED;
    }

    ErrorCode_e err = write_register8(PCA9685_MODE1, MODE1_RESTART);
    if (err != OK) {
        return err;
    }

    sleep_ms(PCA9685_RESET_DELAY_MS);
    return OK;
}

ErrorCode_e pca9685_set_auto_increment(bool enable) {
    if (!driver.initialized) {
        return ERR_DRIVER_NOT_INITIALIZED;
    }
    uint8_t mode1 = 0U;
    read_register8(PCA9685_MODE1, &mode1);

    uint8_t new_mode = mode1;

    if (enable) {
        new_mode |= MODE1_AI;
    } else {
        new_mode |= ~MODE1_AI;
    }

    driver.mode1_reg_val = new_mode;
    return write_register8(PCA9685_MODE1, new_mode);
}

ErrorCode_e pca9685_set_pwm(uint8_t idx, uint16_t on, uint16_t off) {
    assert(idx < PCA9685_MAX_CHANNELS);
    assert(on <= PCA9685_MAX_PWM_VALUE);
    assert(off <= PCA9685_MAX_PWM_VALUE);

    if (!driver.initialized) {
        return ERR_DRIVER_NOT_INITIALIZED;
    }
    if (idx >= PCA9685_MAX_CHANNELS) {
        return ERR_INVALID_PARAM;
    }
    if ((on > PCA9685_MAX_PWM_VALUE) || (off > PCA9685_MAX_PWM_VALUE)) {
        return ERR_INVALID_PARAM;
    }

    uint8_t reg = PCA9685_LEDSTART + (4U * idx);
    uint8_t buf[PCA9685_MAX_DATA_BYTES];
    buf[0] = (uint8_t)(on);
    buf[1] = (uint8_t)(on >> 8U);
    buf[2] = (uint8_t)(off);
    buf[3] = (uint8_t)(off >> 8U);

    return write_register(driver.i2c, driver.addr, reg, buf, 4);
}

ErrorCode_e pca9685_set_pin(uint8_t idx) {
    assert(idx < PCA9685_MAX_CHANNELS);

    if (!driver.initialized) {
        return ERR_DRIVER_NOT_INITIALIZED;
    }
    if (idx >= PCA9685_MAX_CHANNELS) {
        return ERR_INVALID_PARAM;
    }

    return pca9685_set_pwm(idx, PCA9685_MAX_PWM_VALUE, 0U);
}

ErrorCode_e pca9685_set_output_mode(bool totem) {
    if (!driver.initialized) {
        return ERR_DRIVER_NOT_INITIALIZED;
    }
    uint8_t mode2 = 0U;
    read_register8(PCA9685_MODE2, &mode2);

    if (totem) {
        return write_register8(PCA9685_MODE2, mode2 | MODE2_OUTDRV);
    }
    return write_register8(PCA9685_MODE2, (mode2 & (~MODE2_OUTDRV)));
}

ErrorCode_e pca9685_set_output_invert(bool invert) {
    if (!driver.initialized) {
        return ERR_DRIVER_NOT_INITIALIZED;
    }
    uint8_t mode2 = 0U;
    read_register8(PCA9685_MODE2, &mode2);

    if (invert) {
        return write_register8(PCA9685_MODE2, mode2 | MODE2_INVRT);
    }
    return write_register8(PCA9685_MODE2, mode2 & (~MODE2_INVRT));
}
