# SPDX-License-Identifier: Apache-2.0
# NV-Center Quantum Controller Firmware
# Target: Teensy 4.1 (NXP i.MX RT1062)

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nv_center_controller)

target_sources(app PRIVATE
    src/main.c
    src/sequencer.c
    src/event_table.c
    src/usb_comm.c
    src/adc_sync.c
    src/xbar_routing.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Enable link-time optimization for timing-critical code
if(CONFIG_LTO)
    set_property(TARGET app PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()
