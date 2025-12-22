#ifndef ERRORS_H
#define ERRORS_H

typedef enum ErrorCode_e {
  OK = 0,
  ERR_ERROR = -1,
  ERR_NO_MEMORY = -2,
  ERR_DRIVER_NOT_INITIALIZED = -3,
  ERR_I2C_ERROR = -4,
  ERR_SENSOR_NOT_CONNECTED = -5,
} ErrorCode_e;

#endif // !ERRORS_H
