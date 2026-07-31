cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME JY901P)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    App/RINGBUF/ringbuf.c
    App/MOTOR/motor.c
    App/TASK/task.c
    App/ENCODER/encoder.c
    App/PID/pid.c
    App/CONTROL/control.c
    App/GRAY/gray.c
    App/UARTDBG/uartdbg.c
    App/LINEFOLLOW/line_follow.c
    App/CMDPARSER/cmd_handler.c
    App/OLED/oled.c
    App/KEY/key.c
    App/QUESTION/question.c
    App/Zhangdatou/fifo.c
    App/Zhangdatou/Emm_V5.c
    App/Zhangdatou/motor_bujin.c
    App/BALANCE/balance.c
    App/CAMERA/camera.c
    App/CAMERA/camera_protocol.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    App/RINGBUF
    App/MOTOR
    App/OLED
    App/TASK
    App/ENCODER
    App/PID
    App/CONTROL
    App/GRAY
    App/UARTDBG
    App/LINEFOLLOW
    App/CMDPARSER
    App/KEY
    App/QUESTION
    App/Zhangdatou
    App/BALANCE
    App/CAMERA
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    -Wl,-u,_printf_float     # 浮点 printf
    -Wl,-u,_scanf_float      # 浮点 scanf

    # Add user defined libraries
)

set(CMAKE_C_FLAGS_RELEASE "-Os -g0")   # 发布：最小 + 去调试符号
