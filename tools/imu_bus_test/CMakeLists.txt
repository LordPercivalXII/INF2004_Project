cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(imu_bus_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(imu_bus_test main.c)
target_link_libraries(imu_bus_test pico_stdlib hardware_i2c)
target_compile_options(imu_bus_test PRIVATE -Wall -Wextra -Werror)
pico_enable_stdio_usb(imu_bus_test 1)
pico_enable_stdio_uart(imu_bus_test 0)
pico_add_extra_outputs(imu_bus_test)
