# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.22.1)
project(stepit_hardware_tests VERSION 0.1.0 LANGUAGES CXX)

# This module provides installation directories as per the GNU coding standards.
include(GNUInstallDirs)

# Enable output of compile commands during generation.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
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
find_package(stepit_driver REQUIRED)
find_package(cobs_serial REQUIRED)

add_executable(get_status
    src/get_status.cpp
    src/command_line_utility.hpp
    src/command_line_utility.cpp
)
target_include_directories(get_status
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(get_status stepit_driver::stepit_driver)
ament_target_dependencies(get_status stepit_driver cobs_serial)

ament_package()
