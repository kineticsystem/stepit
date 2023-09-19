# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.22.1)
project(freezer_hardware_tests VERSION 0.1.0 LANGUAGES CXX)

# This module provides installation directories as per the GNU coding standards.
include(GNUInstallDirs)

# Enable output of compile commands during generation.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(
        -Werror
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wconversion
        -Wsign-conversion
        -Wold-style-cast
        -Wno-gnu-zero-variadic-macro-arguments
    )
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(freezer_driver REQUIRED)
find_package(cobs_serial REQUIRED)

add_executable(execute_sequence
    src/execute_sequence.cpp
    src/common_utils.hpp
    src/command_line_utility.hpp
    src/command_line_utility.cpp
)
ament_target_dependencies(execute_sequence freezer_driver cobs_serial)

add_executable(echo
    src/echo.cpp
    src/common_utils.hpp
    src/command_line_utility.hpp
    src/command_line_utility.cpp
)
ament_target_dependencies(echo cobs_serial)

ament_package()
