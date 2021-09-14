cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_STANDARD 14)
include(pico_sdk_import.cmake)
project(Telemetrix4RpiPico)
pico_sdk_init()
add_executable(Telemetrix4RpiPico
        Telemetrix4RpiPico.c
)

pico_generate_pio_header(Telemetrix4RpiPico ${CMAKE_CURRENT_LIST_DIR}/Telemetrix4RpiPico.pio archive/hc_sr04.pio)
pico_enable_stdio_usb(Telemetrix4RpiPico 1)
pico_add_extra_outputs(Telemetrix4RpiPico)
target_link_libraries(Telemetrix4RpiPico pico_stdlib hardware_pwm
        pico_unique_id hardware_watchdog hardware_adc hardware_i2c
        hardware_pio hardware_clocks hardware_spi)

add_executable(pio_Telemetrix4RpiPico archive/x.cpp)
# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(pio_Telemetrix4RpiPico ${CMAKE_CURRENT_LIST_DIR}/Telemetrix4RpiPico.pio archive/hc_sr04.pio
        OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

target_sources(pio_Telemetrix4RpiPico PRIVATE Telemetrix4RpiPico.c)
pico_add_extra_outputs(pio_Telemetrix4RpiPico)

add_custom_target(pio_Telemetrix4RpiPico_datasheet DEPENDS ${CMAKE_CURRENT_LIST_DIR}/generated/Telemetrix4RpiPico.py)
add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/generated/Telemetrix4RpiPico.py
        DEPENDS Telemetrix4RpiPico.pio archive/hc_sr04.pio
        COMMAND Pioasm -o python Telemetrix4RpiPico.pio archive/hc_sr04.pio ./generated/Telemetrix4RpiPico
        )
add_dependencies(pio_Telemetrix4RpiPico pio_Telemetrix4RpiPico_datasheet)
