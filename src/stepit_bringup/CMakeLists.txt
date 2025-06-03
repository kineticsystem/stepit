# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.22.1)
project(stepit_bringup LANGUAGES CXX)
include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
