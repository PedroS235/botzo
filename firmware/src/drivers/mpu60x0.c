#include "drivers/mpu60x0/mpu60x0.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "drivers/i2c.h"
#include "drivers/mpu60x0/registers.h"
#include "error.h"

const MPU60x0Config_t MPU60X0_DEFAULT_CONFIG = {.accel_range = MPU_ACCEL_RANGE_2G,
                                                .gyro_range = MPU_GYRO_RANGE_250};

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    const MPU60x0Config_t *config;
    bool initialized;
} MPU6050_t;

static MPU6050_t driver = {0};

static int16_t raw_to_i16(uint8_t hi, uint8_t lo);
static uint16_t mpu60x0_accel_lsb_per_g(const MPU60x0AccelRange range);
static uint16_t mpu60x0_gyro_lsb_dps(const MPU60x0GyroRange range);
static int32_t accel_scale_mg(int16_t raw, MPU60x0AccelRange range);
static float accel_scale_g(int16_t raw, MPU60x0AccelRange range);
static int32_t gyro_scale_mdps(int16_t raw, MPU60x0GyroRange range);
static float gyro_scale_dps(int16_t raw, MPU60x0GyroRange range);

ErrorCode_e mpu60x0_init(i2c_inst_t *i2c, uint8_t addr, const MPU60x0Config_t *config) {
    if (i2c == NULL) return ERR_INVALID_PARAM;
    if (driver.initialized) return ERR_DRIVER_ALREADY_INITIALIZED;

    driver.addr = addr;
    driver.i2c = i2c;
    driver.config = config;
    driver.initialized = true;

    return OK;
}

void mpu60x0_deinit() {
    if (driver.initialized) {
        (void)memset(&driver, 0, sizeof(MPU6050_t));
    }
}

ErrorCode_e mpu60x0_is_connected() {
    if (!driver.initialized) return ERR_DRIVER_NOT_INITIALIZED;
    uint8_t reg = WHO_AM_I;
    uint8_t buf = 0U;

    ErrorCode_e err = read_register(driver.i2c, driver.addr, reg, &buf, 1U);
    if (err != OK) return err;

    return ((buf == MPU6050_WHO_AM_I_VALUE) || (buf == MPU6000_WHO_AM_I_VALUE))
               ? OK
               : ERR_ERROR;
}

ErrorCode_e mpu60x0_read_accel_raw(Vec3_t *out) {
    if (!driver.initialized) return ERR_DRIVER_NOT_INITIALIZED;
    if (out == NULL) return ERR_INVALID_PARAM;

    uint8_t reg = ACCEL_XOUT_H;
    uint8_t buf[6] = {0};

    ErrorCode_e err = read_register(driver.i2c, driver.addr, reg, buf, 6U);
    if (err != OK) return err;

    out->x = raw_to_i16(buf[0], buf[1]);
    out->y = raw_to_i16(buf[2], buf[3]);
    out->z = raw_to_i16(buf[4], buf[5]);

    return OK;
}

ErrorCode_e mpu60x0_read_accel(Vec3_t *out) {
    Vec3_t raw = {0};
    mpu60x0_read_accel_raw(&raw);

    out->x = accel_scale_mg(raw.x, driver.config->accel_range);
    out->y = accel_scale_mg(raw.y, driver.config->accel_range);
    out->z = accel_scale_mg(raw.z, driver.config->accel_range);

    return OK;
}

ErrorCode_e mpu60x0_read_gyro_raw(Vec3_t *out) {
    if (!driver.initialized) return ERR_DRIVER_NOT_INITIALIZED;
    if (out == NULL) return ERR_INVALID_PARAM;

    uint8_t reg = GYRO_XOUT_H;
    uint8_t buf[6] = {0};

    ErrorCode_e err = read_register(driver.i2c, driver.addr, reg, buf, 6U);
    if (err != OK) return err;

    out->x = raw_to_i16(buf[0], buf[1]);
    out->y = raw_to_i16(buf[2], buf[3]);
    out->z = raw_to_i16(buf[4], buf[5]);

    return OK;
}

ErrorCode_e mpu60x0_read_gyro(Vec3_t *out) {
    Vec3_t raw = {0};
    mpu60x0_read_gyro_raw(&raw);

    out->x = gyro_scale_dps(raw.x, driver.config->gyro_range);
    out->y = gyro_scale_dps(raw.y, driver.config->gyro_range);
    out->z = gyro_scale_dps(raw.z, driver.config->gyro_range);

    return OK;
}

ErrorCode_e mpu60x0_read_temp(float *temp) {
    if (!driver.initialized) return ERR_DRIVER_NOT_INITIALIZED;
    if (temp == NULL) return ERR_INVALID_PARAM;

    uint8_t reg = TEMP_OUT_H;
    uint8_t buf[2] = {0};

    ErrorCode_e err = read_register(driver.i2c, driver.addr, reg, buf, 2U);
    if (err != OK) return err;

    int16_t x = raw_to_i16(buf[0], buf[1]);

    *temp = (float)x / 340.0F + 36.53F;

    return OK;
}

static int16_t raw_to_i16(uint8_t hi, uint8_t lo) {
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

static uint16_t mpu60x0_accel_lsb_per_g(const MPU60x0AccelRange range) {
    switch (range) {
        case MPU_ACCEL_RANGE_2G:
            return 16384U;
        case MPU_ACCEL_RANGE_4G:
            return 8192U;
        case MPU_ACCEL_RANGE_8G:
            return 4096U;
        case MPU_ACCEL_RANGE_16G:
            return 2048U;
    }
    return 16384.0F;
}

static uint16_t mpu60x0_gyro_lsb_dps(const MPU60x0GyroRange range) {
    switch (range) {
        case MPU_GYRO_RANGE_250:
            return 1310;
        case MPU_GYRO_RANGE_500:
            return 655;
        case MPU_GYRO_RANGE_1000:
            return 328;
        case MPU_GYRO_RANGE_2000:
            return 164;
    }
    return 131.0F;
}

static int32_t accel_scale_mg(int16_t raw, MPU60x0AccelRange range) {
    return ((int32_t)raw * 1000) / mpu60x0_accel_lsb_per_g(range);
}

static float accel_scale_g(int16_t raw, MPU60x0AccelRange range) {
    return (float)raw / (float)mpu60x0_accel_lsb_per_g(range);
}

static int32_t gyro_scale_mdps(int16_t raw, MPU60x0GyroRange range) {
    return ((int32_t)raw * 10000) / mpu60x0_gyro_lsb_dps(range);
}

static float gyro_scale_dps(int16_t raw, MPU60x0GyroRange range) {
    return ((float)raw * 10) / (float)mpu60x0_gyro_lsb_dps(range);
}
