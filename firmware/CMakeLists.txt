cmake_minimum_required(VERSION 4.2)

# NOTE: This include must be define before project
include(cmake/pico_sdk_import.cmake)

project(botzo_firmware)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

include_directories(${PROJECT_SOURCE_DIR}/include)

set(SOURCES 
    src/drivers/pca9685.c
)

add_executable(firmware src/main.c ${SOURCES})

pico_enable_stdio_usb(firmware 1)
pico_enable_stdio_uart(firmware 1)
pico_add_extra_outputs(firmware)

target_link_libraries(firmware pico_stdlib hardware_i2c)
target_compile_definitions(firmware PRIVATE
    PICO_DEFAULT_I2C_SDA_PIN=12
    PICO_DEFAULT_I2C_SCL_PIN=13
)
