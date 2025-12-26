#ifndef DRIVERS_PCA9685_H
#define DRIVERS_PCA9685_H

#include <hardware/i2c.h>
#include <stdbool.h>
#include <stdint.h>

#include "error.h"

#define PCA9685_DEFAULT_I2C_ADDRESS 0x40U

typedef struct {
    float freq;
} PCA9685Config_s;

/**
 * @brief Initialize PCA9685 driver
 *
 * @param i2c I2C instance
 * @param addr I2C device address
 * @param config Driver configuration
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e pca9685_init(i2c_inst_t *i2c, uint8_t addr, const PCA9685Config_s *config);

/**
 * @brief Deinitialize PCA9685 driver and release resources
 */
void pca9685_deinit(void);

/**
 * @brief Check if PCA9685 device is connected
 *
 * @return ErrorCode_e OK if connected, error code otherwise
 */
ErrorCode_e pca9685_is_connected(void);

/**
 * @brief Put PCA9685 into low-power sleep mode
 *
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e pca9685_sleep(void);

/**
 * @brief Wake PCA9685 from sleep mode
 *
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e pca9685_wakeup(void);

/**
 * @brief Reset PCA9685 to default state
 *
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e pca9685_reset(void);

/**
 * @brief Get current prescale value
 *
 * @param prescale Output prescale value
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e pca9685_get_prescale(uint8_t *prescale);

/**
 * @brief Get current PWM frequency
 *
 * @param kreq Output frequency value
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e pca9685_get_frequency(uint8_t *kreq);

/**
 * @brief Set prescale value
 *
 * Device must be in sleep mode before calling this function
 *
 * @param prescale Prescale value (3-255)
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e pca9685_set_prescale(uint8_t prescale);

/**
 * @brief Set PWM on/off values for channel
 *
 * @param idx Channel index (0-15)
 * @param on On time (0-4096)
 * @param off Off time (0-4096)
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e pca9685_set_pwm(uint8_t idx, uint16_t on, uint16_t off);

/**
 * @brief Set channel to full on state
 *
 * @param idx Channel index (0-15)
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e pca9685_set_pin(uint8_t idx);

/**
 * @brief Set PWM frequency
 *
 * @param freq Frequency in Hz
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e pca9685_set_frequency(float freq);

/**
 * @brief Enable or disable register auto-increment
 *
 * @param enable True to enable, false to disable
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e pca9685_set_auto_increment(bool enable);

/**
 * @brief Set output driver mode
 *
 * @param totem True for totem pole mode, false for open drain
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e pca9685_set_output_mode(bool totem);

/**
 * @brief Invert output logic levels
 *
 * @param invert True to invert, false for normal
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e pca9685_set_output_invert(bool invert);

#endif  // !DRIVERS_PCA9685_H
