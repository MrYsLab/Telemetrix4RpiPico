cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)
project(Telemetrix4RpiPico)
pico_sdk_init()
add_executable(Telemetrix4RpiPico
        Telemetrix4RpiPico.c
)
pico_enable_stdio_usb(Telemetrix4RpiPico 1)
pico_enable_stdio_uart(Telemetrix4RpiPico 1)
pico_add_extra_outputs(Telemetrix4RpiPico)
target_link_libraries(Telemetrix4RpiPico pico_stdlib hardware_pwm
        pico_unique_id hardware_watchdog hardware_adc)

