# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Pico SDK board target")
include(${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake)

project(inf2004_robot_car C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

option(ROBOT_SIMULATION "Use deterministic simulated sensor data" ON)
if(NOT DEFINED ENV{FREERTOS_KERNEL_PATH})
    message(FATAL_ERROR "Set FREERTOS_KERNEL_PATH to an official FreeRTOS-Kernel checkout")
endif()
set(FREERTOS_KERNEL_PATH "$ENV{FREERTOS_KERNEL_PATH}" CACHE PATH "FreeRTOS-Kernel source")
set(FREERTOS_PORT GCC_ARM_CM0 CACHE STRING "FreeRTOS Cortex-M0+ port")
set(FREERTOS_HEAP 4 CACHE STRING "FreeRTOS heap implementation")

pico_sdk_init()

add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(freertos_config INTERFACE projCOVERAGE_TEST=0)

add_subdirectory(${FREERTOS_KERNEL_PATH} ${CMAKE_BINARY_DIR}/FreeRTOS-Kernel)
target_include_directories(freertos_kernel PUBLIC ${CMAKE_CURRENT_LIST_DIR})

add_executable(robot_car
    src/main.c
    src/gpio_irq_router.c
    src/controller/controller.c
    src/buddy1_telemetry/buddy1_telemetry.c
    src/buddy2_motion/buddy2_motion.c
    src/buddy3_line_barcode/buddy3_line_barcode.c
    src/buddy3_line_barcode/code39.c
    src/buddy4_imu/buddy4_imu.c
    src/buddy5_obstacle/buddy5_obstacle.c
)

target_include_directories(robot_car PRIVATE include ${CMAKE_CURRENT_LIST_DIR})
target_compile_definitions(robot_car PRIVATE ROBOT_SIMULATION=$<BOOL:${ROBOT_SIMULATION}>)
target_compile_options(robot_car PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_link_libraries(robot_car
    pico_stdlib
    hardware_pwm
    hardware_i2c
    hardware_adc
    hardware_gpio
    hardware_timer
    hardware_irq
    freertos_kernel
)

pico_enable_stdio_usb(robot_car 1)
pico_enable_stdio_uart(robot_car 0)
pico_add_extra_outputs(robot_car)

add_executable(wifi_smoke_test tools/wifi_smoke_test/wifi_smoke_test.c)
target_include_directories(wifi_smoke_test PRIVATE tools/wifi_smoke_test)
target_link_libraries(wifi_smoke_test pico_stdlib pico_cyw43_arch_lwip_threadsafe_background)
pico_enable_stdio_usb(wifi_smoke_test 1)
pico_enable_stdio_uart(wifi_smoke_test 0)
pico_add_extra_outputs(wifi_smoke_test)

# Buddy 1 MQTT smoke test
add_executable(mqtt_smoke_test
    tools/wifi_smoke_test/mqtt_smoke_test.c
)

target_include_directories(mqtt_smoke_test PRIVATE
    tools/wifi_smoke_test
)

target_link_libraries(mqtt_smoke_test
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
)

pico_enable_stdio_usb(mqtt_smoke_test 1)
pico_enable_stdio_uart(mqtt_smoke_test 0)
pico_add_extra_outputs(mqtt_smoke_test)
