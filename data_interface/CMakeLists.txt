# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

# IMPORTANT: the code does not compile with C++20.
# See https://bytemeta.vip/repo/ros2/rclcpp/issues/1936
# A fix has been already applied on the next release of Humble.

cmake_minimum_required(VERSION 3.22.1)
project(data_interface VERSION 0.1.0 LANGUAGES CXX)
include(GNUInstallDirs)

# Enable output of compile commands during generation.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# All the dependencies required to build this library. These dependencies
# will also be exported using ament_export_dependencies to help a client
# software find all required transitive dependencies to use this library.
set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    serial
)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Werror -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast -Wno-gnu-zero-variadic-macro-arguments)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)

# Build data_interface library.

# CMake does not need headers listed here and it is generally recommended not
# to add them to avoid additional maintenance burden.
# Unfortunately, QtCreator needs them to display them in the class view,
# for easily navigating between class and header files (F4) and for automatic
# code formatting.
# The time spent in the IDE is longer than the time spend maintaining this
# list, so we added the header files too.
add_library(
    data_interface
    SHARED
    include/data_interface/buffer.hpp
    include/data_interface/crc_utils.hpp
    include/data_interface/data_handler.hpp
    include/data_interface/data_interface.hpp
    include/data_interface/data_utils.hpp
    include/data_interface/serial_exception.hpp
    include/data_interface/serial_handler.hpp
    include/data_interface/serial_interface.hpp
    src/crc_utils.cpp
    src/data_handler.cpp
    src/data_utils.cpp
    src/serial_handler.cpp
)
target_include_directories(
    data_interface
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
    data_interface
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_compile_features(data_interface
    PUBLIC cxx_std_17
)

###############################################################################
# EXPORTS

# This is necessary to allow this library’s clients to use the syntax
# target_link_libraries(client <pkg>::<target_name>)
# target_link_libraries(client data_interface::data_interface)
# Without this a client cannot find this library.
# It can take an arbitrary list of targets named as EXPORT in an install call.
ament_export_targets(
    data_interface_targets # Must match the EXPORT label below.
)
# Help downstream packages to find transitive dependencies i.e. export all
# dependencies required by a package to use this library.
# When a package calls find_package(data_interface), CMake looks for a file
# called data_interfaceConfig.cmake which sets up everything another project
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
    data_interface
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
    TARGETS data_interface
    EXPORT data_interface_targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}      # bin
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} # include
)

###############################################################################
# TESTS

# CTest module automatically creates a BUILD_TESTING option that selects
# whether to enable testing support (ON by default).
include(CTest)
if (BUILD_TESTING)
    add_subdirectory(tests)
endif()

ament_package()

###############################################################################
# LINTERS

add_custom_target(format
    COMMAND clang-format -i `git ls-files *.hpp *.cpp`
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
add_custom_target(tidy
    COMMAND clang-tidy -p ${CMAKE_BINARY_DIR} `git ls-files *.cpp`
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
