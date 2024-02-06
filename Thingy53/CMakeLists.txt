#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

set(PM_STATIC_YML_FILE
  ${CMAKE_CURRENT_SOURCE_DIR}/boards/pm_static_${BOARD}.yml
  )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project("Power Controller" VERSION 0.1)

###


include(utils/cmake/utils.cmake)

# Add the Edge Impulse SDK
add_subdirectory(ei-model/edge-impulse-sdk/cmake/zephyr)


# Edge impulse SDK include directories
set(INCLUDES
    .
    ei-model
    ei-model/tflite-model
    ei-model/model-parameters
    src
    )
include_directories(${INCLUDES})

RECURSIVE_FIND_FILE(MODEL_FILES "ei-model/tflite-model" "*.cpp")

list(APPEND SOURCE_FILES ${MODEL_FILES})

###

# NORDIC SDK APP START
target_sources(app PRIVATE ${SOURCE_FILES})
target_sources(app PRIVATE src/main.cpp)

# Preinitialization related to Thingy:53 DFU
target_sources_ifdef(CONFIG_BOARD_THINGY53_NRF5340_CPUAPP app PRIVATE
  boards/thingy53.c
)

# NORDIC SDK APP END
zephyr_library_include_directories(.)
