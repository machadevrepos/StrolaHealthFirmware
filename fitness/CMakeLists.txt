cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nrf7002_bmi270_steps)

target_sources(app PRIVATE
    src/main.c
    src/bmi2.c
    src/bmi270.c
)

target_include_directories(app PRIVATE src)