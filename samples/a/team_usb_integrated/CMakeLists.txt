cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(team_usb_integrated)

target_sources(app PRIVATE
    src/main.c
    src/usb_control.c
    src/module_r1_catch.c
    src/module_lift.c
    src/module_sbus2.c
)
