# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

# IMPORTANT: the code does not compile with C++20.
# See https://bytemeta.vip/repo/ros2/rclcpp/issues/1936
# A fix has been already applied on the next release of Humble.

cmake_minimum_required(VERSION 3.22.1)
project(stepit_hardware VERSION 0.1.0 LANGUAGES CXX)
include(GNUInstallDirs)

# Enable output of compile commands during generation.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Werror -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast)
    add_compile_options($<$<CONFIG:DEBUG>:-fsanitize=address,undefined>)
    add_link_options($<$<CONFIG:DEBUG>:-fsanitize=address,undefined>)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial REQUIRED)

# Build stepit_hardware library.

# QCreator would need header files listed here to display them in the class view,
# but it is not considered good practice for maintenance reasons.
add_library(
  stepit_hardware
  SHARED
  src/request.cpp
  src/response.cpp
  src/motor_move_to_command.cpp
  src/motor_status.cpp
  src/data_utils.cpp
  src/data_buffer.cpp
  src/data_interface.cpp
  src/crc_utils.cpp
  src/serial_interface.cpp
  src/stepit_hardware.cpp
)
target_include_directories(
  stepit_hardware
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  stepit_hardware
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_compile_features(stepit_hardware
  PUBLIC cxx_std_17
)

#add_executable(my_exe
#  src/main.cpp
#  src/data_utils.cpp
#  include/stepit_hardware/data_utils.hpp
#  src/serial_interface.cpp
#  include/stepit_hardware/serial_interface.hpp
#)
#target_include_directories(
#  my_exe
#  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  PUBLIC $<INSTALL_INTERFACE:include>)
#ament_target_dependencies(my_exe
#  rclcpp
#  serial
#)

#target_compile_features(stepit_hardware
#  PUBLIC cxx_std_17
#)

###############################################################################
# PLUGINS

pluginlib_export_plugin_description_file(hardware_interface stepit_hardware.xml)

###############################################################################
# INSTALL

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  COMPONENT stepit_hardware_headers
)
install(
  TARGETS stepit_hardware
  EXPORT stepit_hardware_targets
  COMPONENT stepit_hardware_cmake
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

###############################################################################
# EXPORTS

ament_export_include_directories(
  include
)
ament_export_libraries(
  stepit_hardware
)
ament_export_targets(
  stepit_hardware_targets
)

## Export all dependencies required by a package to use this library.
## When a package calls find_package(stepit_hardware), CMake looks for a file
## called stepit_hardwareConfig.cmake which sets up everything another project
## would need to depend on this one.
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

###############################################################################
# TESTS

include(CTest)
if (BUILD_TESTING)
    add_subdirectory(tests)
endif()

ament_package()
