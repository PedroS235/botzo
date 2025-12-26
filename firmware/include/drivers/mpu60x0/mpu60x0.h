#ifndef DRIVERS_MPU60x0_H
#define DRIVERS_MPU60x0_H

#include <hardware/i2c.h>
#include <stdint.h>

#include "error.h"

#define MPU60X0_DEFAULT_I2C_ADDRESS 0x68

typedef enum {
    MPU_ACCEL_RANGE_2G,
    MPU_ACCEL_RANGE_4G,
    MPU_ACCEL_RANGE_8G,
    MPU_ACCEL_RANGE_16G,
} MPU60x0AccelRange;

typedef enum {
    MPU_GYRO_RANGE_250,
    MPU_GYRO_RANGE_500,
    MPU_GYRO_RANGE_1000,
    MPU_GYRO_RANGE_2000,
} MPU60x0GyroRange;

typedef struct {
    MPU60x0AccelRange accel_range;
    MPU60x0GyroRange gyro_range;
} MPU60x0Config_t;

typedef struct {
    int x;
    int y;
    int z;
} Vec3_t;

extern const MPU60x0Config_t MPU60X0_DEFAULT_CONFIG;

/**
 * @brief Initialize MPU60x0 driver
 *
 * @param i2c I2C instance
 * @param addr I2C device address
 * @param config Driver configuration
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e mpu60x0_init(i2c_inst_t *i2c, uint8_t addr, const MPU60x0Config_t *config);

/**
 * @brief Deinitialize MPU60x0 driver and release resources
 */
void mpu60x0_deinit(void);

/**
 * @brief Check if MPU60x0 device is connected
 *
 * @return ErrorCode_e OK if connected, error code otherwise
 */
ErrorCode_e mpu60x0_is_connected();

/**
 * @brief Read scaled accelerometer data
 *
 * @param out Output vector in mg (milligravity)
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e mpu60x0_read_accel(Vec3_t *out);

/**
 * @brief Read raw accelerometer data
 *
 * @param out Output vector with raw values
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e mpu60x0_read_accel_raw(Vec3_t *out);

/**
 * @brief Read raw gyroscope data
 *
 * @param out Output vector with raw values
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e mpu60x0_read_gyro_raw(Vec3_t *out);

/**
 * @brief Read scaled gyroscope data
 *
 * @param out Output vector in dps (degrees per second)
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e mpu60x0_read_gyro(Vec3_t *out);

/**
 * @brief Read temperature sensor
 *
 * @param temp Output temperature in degrees Celsius
 * @return ErrorCode_e OK on success, error code otherwise
 */
ErrorCode_e mpu60x0_read_temp(float *temp);

#endif  // !DRIVERS_MPU60x0_H
