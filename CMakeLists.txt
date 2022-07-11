# Copyright (c) 2021 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.19.0)

set(MBED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/mbed-os CACHE INTERNAL "")
set(MBED_CONFIG_PATH ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
set(APP_TARGET Application)

include(${MBED_PATH}/tools/cmake/app.cmake)

project(${APP_TARGET})


add_subdirectory(${MBED_PATH})
add_subdirectory(MS5611_01BA03)
add_subdirectory(ICM_20948)
add_subdirectory(INA219)
add_subdirectory(HTU21D)
add_subdirectory(Honeywell_RSC)
add_subdirectory(lib)

add_executable(${APP_TARGET}
    main.cpp
	net_com.cpp
	#imu_task.cpp
)

target_link_libraries(${APP_TARGET} mbed-os mbed-netsocket mbed-emac mbed-usb mbed-stm mbed-stm32f7 mbed-stm32f746xg mbed-sensorboard mbed-core mbed-rtos mbed-lwipstack)
target_link_libraries(${APP_TARGET} MS5611_01BA03)
target_link_libraries(${APP_TARGET} ICM_20948)
target_link_libraries(${APP_TARGET} INA219)
target_link_libraries(${APP_TARGET} HTU21D)
target_link_libraries(${APP_TARGET} Honeywell_RSC)

mbed_set_post_build(${APP_TARGET})

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()