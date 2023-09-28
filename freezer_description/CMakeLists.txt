cmake_minimum_required(VERSION 3.22.1)
project(freezer_description VERSION 0.1.0 LANGUAGES CXX)

# This module provides installation directories as per the GNU coding standards.
include(GNUInstallDirs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)

###############################################################################
# INSTALL

install(DIRECTORY
  config
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
