# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

# IMPORTANT: the code does not compile with C++20.
# See https://bytemeta.vip/repo/ros2/rclcpp/issues/1936
# A fix has been already applied on the next release of Humble.

cmake_minimum_required(VERSION 3.22.1)
project(stepit_teleop VERSION 0.1.0 LANGUAGES CXX)
include(GNUInstallDirs)

# Enable output of compile commands during generation.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_lifecycle
    std_msgs
    geometry_msgs
)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Werror -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast -Wno-gnu-zero-variadic-macro-arguments)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build stepit_teleop app.

# CMake does not need headers listed here and it is generally recommended not
# to add them to avoid additional maintenance burdens.
# Unfortunately, QtCreator needs them, to display them in the class view,
# for easily navigating between class and header files (F4) and for automatic
# code formatting.
add_executable(
    velocity_teleop_node
    src/velocity_teleop_main.cpp
    src/velocity_teleop_node.cpp
    include/stepit_teleop/velocity_teleop_node.hpp
)
target_include_directories(
    velocity_teleop_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# This is used by the linked to find all includes required by the target.
ament_target_dependencies(
    velocity_teleop_node
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_compile_features(velocity_teleop_node
    PUBLIC cxx_std_17
)

###############################################################################
# INSTALL


install(
  TARGETS
    velocity_teleop_node
  DESTINATION lib/stepit_teleop)

# Install directories
install(
  DIRECTORY
    config
    launch
  DESTINATION share/stepit_teleop
)

ament_package()
