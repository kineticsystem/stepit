# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

# IMPORTANT: the code does not compile with C++20.
# See https://bytemeta.vip/repo/ros2/rclcpp/issues/1936
# A fix has been already applied on the next release of Humble.

cmake_minimum_required(VERSION 3.22.1)
project(stepit_control VERSION 0.1.0 LANGUAGES CXX)
include(GNUInstallDirs)

# Enable output of compile commands during generation.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# All the dependencies required to build this library. These dependencies
# will also be exported using ament_export_dependencies to help a client
# software find all required transitive dependencies to use this library.
set(THIS_PACKAGE_INCLUDE_DEPENDS
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    serial
)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Werror -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast -Wno-gnu-zero-variadic-macro-arguments)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial REQUIRED)

# Build stepit_control library.

# CMake does not need headers listed here and it is generally recommended not
# to add them to avoid additional maintenance burden.
# Unfortunately, QtCreator needs them, to display them in the class view,
# for easily navigating between class and header files (F4) and for automatic
# code formatting.
add_library(
    stepit_control
    SHARED
    src/msgs/request.cpp
    include/stepit_control/msgs/request.hpp
    src/msgs/response.cpp
    include/stepit_control/msgs/response.hpp
    src/data_utils.cpp
    include/stepit_control/data_utils.hpp
    src/data_handler.cpp
    include/stepit_control/data_handler.hpp
    src/crc_utils.cpp
    include/stepit_control/crc_utils.hpp
    src/serial_handler.cpp
    include/stepit_control/serial_handler.hpp
    src/stepit_control.cpp
    include/stepit_control/stepit_control.hpp
    include/stepit_control/buffer.hpp
    include/stepit_control/data_interface.hpp
    include/stepit_control/serial_interface.hpp
    include/stepit_control/serial_exception.hpp
    include/stepit_control/msgs/msgs_ids.hpp
    src/msgs/motor_status_query.cpp
    include/stepit_control/msgs/motor_status_query.hpp
    src/msgs/motor_status_response.cpp
    include/stepit_control/msgs/motor_status_response.hpp
    src/msgs/motor_velocity_command.cpp
    include/stepit_control/msgs/motor_velocity_command.hpp
    src/msgs/motor_position_command.cpp
    include/stepit_control/msgs/motor_position_command.hpp
    src/msgs/acknowledge_response.cpp
    include/stepit_control/msgs/acknowledge_response.hpp
    src/fake/position_control.cpp
    include/stepit_control/fake/position_control.hpp
    src/fake/velocity_control.cpp
    include/stepit_control/fake/velocity_control.hpp
    include/stepit_control/command_interface.hpp
    src/command_handler.cpp
    include/stepit_control/command_handler.hpp
    src/fake/fake_command_handler.cpp
    include/stepit_control/fake/fake_command_handler.hpp
    src/fake/fake_motor.cpp
    include/stepit_control/fake/fake_motor.hpp
    src/msgs/motor_config_command.cpp
    include/stepit_control/msgs/motor_config_command.hpp
    include/stepit_control/msgs/msgs.hpp
)
target_include_directories(
    stepit_control
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
    stepit_control
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_compile_features(stepit_control
    PUBLIC cxx_std_17
)

###############################################################################
# PLUGINS

pluginlib_export_plugin_description_file(hardware_interface stepit_control.xml)

###############################################################################
# EXPORTS

# This is necessary to allow this library’s clients to use the syntax
# target_link_libraries(client <pkg>::<target_name>)
# target_link_libraries(client stepit_control::stepit_control)
# Without this a client cannot find this library.
# It can take an arbitrary list of targets named as EXPORT in an install call.
ament_export_targets(
    stepit_control_targets # Must match the EXPORT label below.
)
# Help downstream packages to find transitive dependencies i.e. export all
# dependencies required by a package to use this library.
# When a package calls find_package(stepit_control), CMake looks for a file
# called stepit_controlConfig.cmake which sets up everything another project
# would need to depend on this one.
ament_export_dependencies(
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# Tell downstream packages where to find our headers.
ament_export_include_directories(
    include
)
# Tell downstream packages our libraries to link against.
ament_export_libraries(
    stepit_control
)

###############################################################################
# INSTALL

# Install all files of the include folder into the give destination.
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}          # include
)

# Install our library.
install(
    TARGETS stepit_control
    EXPORT stepit_control_targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}      # bin
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} # include
)


###############################################################################
# TESTS

include(CTest)
if (BUILD_TESTING)
    add_subdirectory(tests)
endif()

ament_package()
