#include "drivers/pca9685/pca9685.h"

#include <assert.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <pico/error.h>
#include <pico/time.h>
#include <pico/types.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
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

// Oscillator calibration constants
#define CALIB_MAX_SAMPLES 10
#define CALIB_SAMPLE_INTERVAL_US 1000000UL  // 1 second per sample
#define CALIB_DUTY_CYCLE_50PCT 2048U        // Half of 4096 for 50% duty
#define US_PER_SECOND 1000000UL
#define MS_PER_SECOND 1000UL

#define DEFAULT_MODE1 0x11U

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    float oscclock;
    uint8_t mode1_reg_val;
    const PCA9685Config_t *config;
    bool initialized;
} PCA9685_t;

typedef struct {
    uint8_t feedback_gpio;
    uint8_t feedback_idx;
    uint32_t real_osc_freq;
    uint8_t prescale;

    absolute_time_t total_time_us;
    absolute_time_t prev_time;
    absolute_time_t interval;

    uint16_t interval_counter;
    uint32_t total_counter;
    gpio_irq_callback_t irq_handler;
} auto_calib_state_t;

static PCA9685_t driver = {0};
static auto_calib_state_t auto_calib_state = {0};

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

ErrorCode_e pca9685_init(i2c_inst_t *i2c, uint8_t addr, const PCA9685Config_t *config) {
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

ErrorCode_e pca9685_get_us_per_count(float *us_per_count) {
    assert(us_per_count != NULL);

    if (!driver.initialized) {
        return ERR_DRIVER_NOT_INITIALIZED;
    }

    uint8_t prescale = 0U;
    ErrorCode_e err = pca9685_get_prescale(&prescale);
    if (err != OK) {
        return err;
    }

    // Calculate microseconds per counter tick
    // pulselength = 1000000 * (prescale + 1) / oscclock
    *us_per_count = 1000000.0f * (prescale + 1U) / driver.oscclock;

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

    float prescaleval = ((driver.oscclock / (freq * 4096.0F)) + 0.5F) - 1.0F;
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
    ErrorCode_e err = read_register8(PCA9685_MODE1, &mode1);
    if (err != OK) {
        return err;
    }

    uint8_t new_mode = mode1;

    if (enable) {
        new_mode |= MODE1_AI;
    } else {
        new_mode &= ~MODE1_AI;
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

ErrorCode_e pca9685_set_pulse_width(uint8_t idx, uint16_t pulse_width_us) {
    assert(idx < PCA9685_MAX_CHANNELS);

    if (!driver.initialized) {
        return ERR_DRIVER_NOT_INITIALIZED;
    }

    uint8_t prescale = 0U;
    ErrorCode_e err = pca9685_get_prescale(&prescale);
    if (err != OK) {
        return err;
    }

    double pulse = (double)pulse_width_us;
    double pulselength = 1000000.0;

    pulselength *= (prescale + 1U);
    pulselength /= driver.oscclock;

    pulse /= pulselength;

    uint16_t counter = (uint16_t)pulse;
    if (counter > PCA9685_MAX_PWM_VALUE) {
        counter = PCA9685_MAX_PWM_VALUE;
    }

    return pca9685_set_pwm(idx, 0U, counter);
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
    ErrorCode_e err = read_register8(PCA9685_MODE2, &mode2);
    if (err != OK) {
        return err;
    }

    if (totem) {
        return write_register8(PCA9685_MODE2, mode2 | MODE2_OUTDRV);
    }
    return write_register8(PCA9685_MODE2, mode2 & ~MODE2_OUTDRV);
}

ErrorCode_e pca9685_set_output_invert(bool invert) {
    if (!driver.initialized) {
        return ERR_DRIVER_NOT_INITIALIZED;
    }
    uint8_t mode2 = 0U;
    ErrorCode_e err = read_register8(PCA9685_MODE2, &mode2);
    if (err != OK) {
        return err;
    }

    if (invert) {
        return write_register8(PCA9685_MODE2, mode2 | MODE2_INVRT);
    }
    return write_register8(PCA9685_MODE2, mode2 & ~MODE2_INVRT);
}

void auto_calibration_irq(uint gpio, uint32_t event_mask) {
    auto_calib_state.interval_counter++;
}

static ErrorCode_e init_calibration_state(uint8_t idx,
                                          uint gpio,
                                          uint8_t prescale,
                                          uint32_t interval_us) {
    auto_calib_state.feedback_gpio = gpio;
    auto_calib_state.feedback_idx = idx;
    auto_calib_state.interval_counter = 0;
    auto_calib_state.total_counter = 0;
    auto_calib_state.total_time_us = 0;
    auto_calib_state.prev_time = get_absolute_time();
    auto_calib_state.interval = interval_us;
    auto_calib_state.real_osc_freq = driver.oscclock;
    auto_calib_state.prescale = prescale;
    auto_calib_state.irq_handler = auto_calibration_irq;
    return OK;
}

static ErrorCode_e setup_calibration_hardware(void) {
    gpio_init(auto_calib_state.feedback_gpio);
    gpio_set_irq_enabled_with_callback(auto_calib_state.feedback_gpio,
                                       GPIO_IRQ_EDGE_RISE,
                                       true,
                                       auto_calib_state.irq_handler);
    ErrorCode_e err =
        pca9685_set_pwm(auto_calib_state.feedback_idx, 0, CALIB_DUTY_CYCLE_50PCT);
    if (err != OK) {
        gpio_deinit(auto_calib_state.feedback_gpio);
        return err;
    }
    return OK;
}

static uint32_t calculate_osc_freq_sample(void) {
    uint64_t real_osc_freq =
        (uint64_t)(auto_calib_state.prescale + 1) * auto_calib_state.total_counter;
    uint64_t multiplier = PCA9685_MAX_PWM_VALUE;

    real_osc_freq *= US_PER_SECOND;

    while (((real_osc_freq & 0x8000000000000000ULL) == 0) && (multiplier != 1)) {
        real_osc_freq <<= 1;
        multiplier >>= 1;
    }
    real_osc_freq /= auto_calib_state.total_time_us;

    if (multiplier) {
        real_osc_freq *= multiplier;
    }
    return (uint32_t)real_osc_freq;
}

static ErrorCode_e collect_calibration_samples(uint32_t samples[CALIB_MAX_SAMPLES]) {
    size_t iteration = 0;
    auto_calib_state.prev_time = get_absolute_time();

    while (iteration < CALIB_MAX_SAMPLES) {
        absolute_time_t now = get_absolute_time();
        absolute_time_t time_diff =
            absolute_time_diff_us(auto_calib_state.prev_time, now);

        if (time_diff > auto_calib_state.interval) {
            uint16_t interval_count = auto_calib_state.interval_counter;
            auto_calib_state.interval_counter = 0;
            auto_calib_state.prev_time = now;

            auto_calib_state.total_counter += interval_count;
            auto_calib_state.total_time_us += time_diff;

            uint32_t freq = calculate_osc_freq_sample();

            samples[iteration] = freq;
            iteration++;
        }
    }
    return OK;
}

static uint32_t average_samples(const uint32_t samples[CALIB_MAX_SAMPLES]) {
    uint64_t sum = 0;
    for (size_t i = 0; i < CALIB_MAX_SAMPLES; i++) {
        sum += samples[i];
    }
    return (uint32_t)(sum / CALIB_MAX_SAMPLES);
}

static ErrorCode_e apply_calibration_result(uint32_t calibrated_freq) {
    driver.oscclock = (float)calibrated_freq;
    printf("Average Real Oscilloscope Frequency: %u\n", calibrated_freq);
    printf("Driver oscclock updated to: %.0f\n", driver.oscclock);
    return pca9685_set_frequency(driver.config->freq);
}

static void cleanup_calibration_hardware(void) {
    gpio_deinit(auto_calib_state.feedback_gpio);
}

ErrorCode_e pca9685_auto_calibrate_osc_blocking_ex(uint8_t idx,
                                                   uint gpio,
                                                   uint32_t interval_us) {
    if (!driver.initialized) {
        return ERR_DRIVER_NOT_INITIALIZED;
    }

    if (interval_us < 50000UL) {
        return ERR_INVALID_PARAM;
    }
    if (interval_us > 5000000UL) {
        return ERR_INVALID_PARAM;
    }

    uint8_t prescale = 0;
    ErrorCode_e err = pca9685_get_prescale(&prescale);
    if (err != OK) {
        return err;
    }

    err = init_calibration_state(idx, gpio, prescale, interval_us);
    if (err != OK) {
        return err;
    }

    err = setup_calibration_hardware();
    if (err != OK) {
        return err;
    }

    uint32_t samples[CALIB_MAX_SAMPLES] = {0};
    err = collect_calibration_samples(samples);
    if (err != OK) {
        cleanup_calibration_hardware();
        return err;
    }

    uint32_t calibrated_freq = average_samples(samples);
    err = apply_calibration_result(calibrated_freq);
    if (err != OK) {
        cleanup_calibration_hardware();
        return err;
    }

    cleanup_calibration_hardware();
    return OK;
}

ErrorCode_e pca9685_auto_calibrate_osc_blocking(uint8_t idx, uint gpio) {
    return pca9685_auto_calibrate_osc_blocking_ex(idx, gpio, CALIB_SAMPLE_INTERVAL_US);
}
