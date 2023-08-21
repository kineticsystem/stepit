# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.22.1)
project(stepit_hardware_tests VERSION 0.1.0 LANGUAGES CXX)

# This module provides installation directories as per the GNU coding standards.
include(GNUInstallDirs)

# Enable output of compile commands during generation.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
ament_target_dependencies(get_status stepit_driver)

ament_package()
