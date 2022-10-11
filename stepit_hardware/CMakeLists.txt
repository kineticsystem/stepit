# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

# IMPORTANT: the code does not compile with C++20.
# See https://bytemeta.vip/repo/ros2/rclcpp/issues/1936
# A fix has been already applied on the next release of Humble.

cmake_minimum_required(VERSION 3.22.1)
project(stepit_hardware VERSION 0.1.0 LANGUAGES CXX)
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

# Build stepit_hardware library.

# CMake does not need headers listed here and it is generally recommended not
# to add them to avoid additional maintenance burden.
# Unfortunately, QtCreator needs them, to display them in the class view,
# for easily navigating between class and header files (F4) and for automatic
# code formatting.
add_library(
    stepit_hardware
    SHARED
    src/msgs/request.cpp
    include/stepit_hardware/msgs/request.hpp
    src/msgs/response.cpp
    include/stepit_hardware/msgs/response.hpp
    src/data_utils.cpp
    include/stepit_hardware/data_utils.hpp
    src/data_handler.cpp
    include/stepit_hardware/data_handler.hpp
    src/crc_utils.cpp
    include/stepit_hardware/crc_utils.hpp
    src/serial_handler.cpp
    include/stepit_hardware/serial_handler.hpp
    src/stepit_hardware.cpp
    include/stepit_hardware/stepit_hardware.hpp
    include/stepit_hardware/buffer.hpp
    include/stepit_hardware/data_interface.hpp
    include/stepit_hardware/serial_interface.hpp
    include/stepit_hardware/serial_exception.hpp
    include/stepit_hardware/msgs/msgs_ids.hpp
    src/msgs/status_query.cpp
    include/stepit_hardware/msgs/status_query.hpp
    src/msgs/status_response.cpp
    include/stepit_hardware/msgs/status_response.hpp
    src/msgs/velocity_command.cpp
    include/stepit_hardware/msgs/velocity_command.hpp
    src/msgs/position_command.cpp
    include/stepit_hardware/msgs/position_command.hpp
    src/msgs/acknowledge_response.cpp
    include/stepit_hardware/msgs/acknowledge_response.hpp
    src/fake/position_control.cpp
    include/stepit_hardware/fake/position_control.hpp
    src/fake/velocity_control.cpp
    include/stepit_hardware/fake/velocity_control.hpp
    include/stepit_hardware/command_interface.hpp
    src/command_handler.cpp
    include/stepit_hardware/command_handler.hpp
    src/fake/fake_command_handler.cpp
    include/stepit_hardware/fake/fake_command_handler.hpp
    src/fake/fake_motor.cpp
    include/stepit_hardware/fake/fake_motor.hpp
    src/msgs/config_command.cpp
    include/stepit_hardware/msgs/config_command.hpp
    include/stepit_hardware/msgs/msgs.hpp
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

###############################################################################
# PLUGINS

pluginlib_export_plugin_description_file(hardware_interface stepit_hardware.xml)

###############################################################################
# EXPORTS

# This is necessary to allow this library’s clients to use the syntax
# target_link_libraries(client <pkg>::<target_name>)
# target_link_libraries(client stepit_hardware::stepit_hardware)
# Without this a client cannot find this library.
# It can take an arbitrary list of targets named as EXPORT in an install call.
ament_export_targets(
    stepit_hardware_targets # Must match the EXPORT label below.
)
# Help downstream packages to find transitive dependencies i.e. export all
# dependencies required by a package to use this library.
# When a package calls find_package(stepit_hardware), CMake looks for a file
# called stepit_hardwareConfig.cmake which sets up everything another project
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
    stepit_hardware
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
    TARGETS stepit_hardware
    EXPORT stepit_hardware_targets
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
