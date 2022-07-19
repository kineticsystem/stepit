# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

# IMPORTANT: the code does not compile with C++20.
# See https://bytemeta.vip/repo/ros2/rclcpp/issues/1936
# A fix has been already applied on the next release of Humble.

cmake_minimum_required(VERSION 3.16.3)
project(stepit_hardware LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
)

# Build stepit_hardware library.

add_library(
  ${PROJECT_NAME} # QCreator needs header files to display them.
  SHARED
  src/request.cpp
  include/stepit_hardware/request.hpp
  src/response.cpp
  include/stepit_hardware/response.hpp
  src/motor_move_to_command.cpp
  include/stepit_hardware/motor_move_to_command.hpp
  src/motor_status.cpp
  include/stepit_hardware/motor_status.hpp
  src/data_utils.cpp
  include/stepit_hardware/data_utils.hpp
  include/stepit_hardware/buffer.hpp
  include/stepit_hardware/buffer_position.hpp
  src/data_buffer.cpp
  include/stepit_hardware/data_buffer.hpp
  src/data_interface.cpp
  include/stepit_hardware/data_interface.hpp
  include/stepit_hardware/serial.hpp
  src/crc_utils.cpp
  include/stepit_hardware/crc_utils.hpp
  src/serial_interface.cpp
  include/stepit_hardware/serial_interface.hpp
  include/stepit_hardware/serial_exception.hpp
  src/stepit_hardware.cpp
  include/stepit_hardware/stepit_hardware.hpp
  include/stepit_hardware/visibility_control.hpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_compile_features(${PROJECT_NAME}
  PRIVATE cxx_std_17
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

#target_compile_features(${PROJECT_NAME}
#  PRIVATE cxx_std_17
#)

###############################################################################
# PLUGINS

pluginlib_export_plugin_description_file(hardware_interface stepit_hardware.xml)

###############################################################################
# TESTS

if (BUILD_TESTING)
    add_subdirectory(test)
endif()

###############################################################################
# INSTALL

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  DIRECTORY include/
  DESTINATION include
)

###############################################################################
# EXPORTS

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

## Export all dependencies required by a package to use this library.
## When a package calls find_package(stepit_hardware), CMake looks for a file
## called stepit_hardwareConfig.cmake which sets up everything another project
## would need to depend on this one.
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
