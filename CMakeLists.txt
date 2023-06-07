cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(PicoJoybus C CXX ASM)

set(PICO_BOARD pico_w)

pico_sdk_init()

include_directories(${CMAKE_CURRENT_LIST_DIR}/src/config)

add_executable(PicoJoybus
    src/main.c
    src/joybus.c
)

pico_generate_pio_header(PicoJoybus ${CMAKE_CURRENT_LIST_DIR}/src/joybus.pio)

target_link_libraries(PicoJoybus
    hardware_dma
    hardware_pio
    # pico_btstack_classic
    pico_btstack_ble
    pico_btstack_cyw43
    # pico_cyw43_arch_threadsafe_background
    pico_cyw43_arch_none
    pico_stdlib
)

# pico_enable_stdio_usb(PicoJoybus 1)
pico_enable_stdio_uart(PicoJoybus 1)

pico_add_extra_outputs(PicoJoybus)
