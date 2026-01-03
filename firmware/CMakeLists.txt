# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

# NOTE: This include must be define before project
include(cmake/pico_sdk_import.cmake)

project(botzo_firmware)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable debug symbols for OpenOCD flashing
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

pico_sdk_init()

include_directories(${PROJECT_SOURCE_DIR}/include)

set(SOURCES
    src/drivers/pca9685.c
    src/drivers/mpu60x0.c
    src/drivers/i2c.c
    src/scheduler.c
    src/servo.c
)

add_executable(firmware src/main.c ${SOURCES})

pico_enable_stdio_usb(firmware 1)
pico_enable_stdio_uart(firmware 1)
pico_add_extra_outputs(firmware)

target_link_libraries(firmware pico_stdlib hardware_i2c pico_status_led hardware_irq)
target_compile_definitions(firmware PRIVATE
    PICO_DEFAULT_I2C_SDA_PIN=12
    PICO_DEFAULT_I2C_SCL_PIN=13
)

# OpenOCD flash target with system reset
add_custom_target(flash
    COMMAND ${USERHOME}/.pico-sdk/openocd/0.12.0+dev/openocd -s ${USERHOME}/.pico-sdk/openocd/0.12.0+dev/scripts -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" -c "program ${CMAKE_BINARY_DIR}/firmware.elf verify" -c "reset run" -c "exit"
    DEPENDS firmware
    COMMENT "Flashing firmware via Debug Probe (power cycle recommended for clean reset)"
)

# OpenOCD reset target
add_custom_target(reset
    COMMAND ${USERHOME}/.pico-sdk/openocd/0.12.0+dev/openocd -s ${USERHOME}/.pico-sdk/openocd/0.12.0+dev/scripts -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" -c "init" -c "reset" -c "exit"
    COMMENT "Resetting target via Debug Probe"
)
