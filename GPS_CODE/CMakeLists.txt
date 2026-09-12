cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico2)
set(PICO_PLATFORM rp2350-arm-s)
include(pico_sdk_import.cmake)
project(ICM42688P C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(ICM42688P  ICM42688P.c)

pico_enable_stdio_usb(ICM42688P 1)
pico_enable_stdio_uart(ICM42688P 0)   # keep Pico UART free for the GPS

target_link_libraries(ICM42688P pico_stdlib hardware_uart hardware_spi)

pico_add_extra_outputs(ICM42688P)
