#ifndef ERRORS_H
#define ERRORS_H

typedef enum ErrorCode_e {
    OK = 0,
    ERR_ERROR = -1,
    ERR_NO_MEMORY = -2,
    ERR_DRIVER_NOT_INITIALIZED = -3,
    ERR_I2C_ERROR = -4,
    ERR_SENSOR_NOT_CONNECTED = -5,
    ERR_INVALID_PARAM = -6,
    ERR_DRIVER_ALREADY_INITIALIZED = -7,
} ErrorCode_e;

#endif  // !ERRORS_H
