# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.20)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

set(AUDIO_OUTPUT 1 CACHE BOOL "0 for serial output, 1 for usb audio output")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(adxl345 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

if (${AUDIO_OUTPUT} EQUAL 1)
    # If AUDIO_OUTPUT is set to 1, use the pico_audio library
    add_executable(adxl345 
        ${PROJECT_SOURCE_DIR}/src/usbaudio.cpp
        ${PROJECT_SOURCE_DIR}/src/adxl_345.cpp
        ${PROJECT_SOURCE_DIR}/src/usb_descriptors.c
    )
else()
    add_executable(adxl345 
        ${PROJECT_SOURCE_DIR}/src/serial.cpp
        ${PROJECT_SOURCE_DIR}/src/adxl_345.cpp
    )
endif()

pico_set_program_name(adxl345 "adxl345")
pico_set_program_version(adxl345 "0.1")

# Modify the below lines to enable/disable output over UART/USB
if (${AUDIO_OUTPUT} EQUAL 1)
    pico_enable_stdio_usb(adxl345 0)
    pico_enable_stdio_uart(adxl345 1)
else()
    pico_enable_stdio_usb(adxl345 1)
    pico_enable_stdio_uart(adxl345 0)
endif()

# Add the standard library to the build
target_link_libraries(adxl345
        pico_stdlib)

# Add the standard include files to the build
if(${AUDIO_OUTPUT} EQUAL 1)
    target_include_directories(adxl345 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${PROJECT_SOURCE_DIR}/include/usbaudio
        ${PROJECT_SOURCE_DIR}/include/serial
    )
else()
    target_include_directories(adxl345 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${PROJECT_SOURCE_DIR}/include/serial
    )
endif()

# Add any user requested libraries
target_link_libraries(adxl345 
        hardware_spi
        hardware_gpio
)

if(${AUDIO_OUTPUT} EQUAL 1)
    target_link_libraries(adxl345 
        tinyusb_device
        tinyusb_board
    )
endif()

pico_add_extra_outputs(adxl345)

