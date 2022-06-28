# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.8)

project(stepit_hardware LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/serial_port.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  src
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
)

ament_package()
