cmake_minimum_required(VERSION 3.12)

#set enviroment and version for cmake
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
set(PICO_SDK_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(app C CXX ASM)
 
# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()


# Add executable. Default name is the project name, version 0.1
 
add_executable(app main.cpp )

pico_set_program_name(app "gesture_reconigtion")
pico_set_program_version(app "0.1")

pico_enable_stdio_uart(app 0)
pico_enable_stdio_usb(app 1)

# Add the standard library to the build
    target_link_libraries(app pico_stdlib)

	# Add the standard include files to the build
    target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
  )
   
# Add any user requested libraries
target_link_libraries(app hardware_i2c)
add_subdirectory(MPU6050_drv)
target_link_libraries(app MPU6050_drv)

pico_add_extra_outputs(app)