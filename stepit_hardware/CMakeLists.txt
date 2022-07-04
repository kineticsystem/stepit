# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.22.1)
project(stepit_hardware LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)

# Build stepit_hardware library.

add_library(${PROJECT_NAME} # QCreator needs header files to display them.
  #src/main.cpp
  src/request.cpp
  src/request.hpp
  src/response.cpp
  src/response.hpp
  src/motor_move_to_command.cpp
  src/motor_move_to_command.hpp
  src/data_utils.cpp
  include/stepit/stepit_hardware/data_utils.hpp
  #src/serial_port.cpp
  #src/serial_port.hpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  serial
)
target_compile_options(${PROJECT_NAME}
  PRIVATE -Wall -Wextra -Wpedantic
)
target_compile_features(${PROJECT_NAME}
  PRIVATE cxx_std_17
)

# Install targets.
install(
  TARGETS
    ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if (BUILD_TESTING)
    # Add support for GTest.
    find_package(ament_cmake_gtest REQUIRED)

    # Add test.
    ament_add_gtest(data_utils_test test/data_utils_test.cpp)
    target_include_directories(data_utils_test
      PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      PUBLIC $<INSTALL_INTERFACE:include>)
    target_compile_options(data_utils_test
      PRIVATE -Wall -Wextra -Wpedantic
    )
    target_link_libraries(data_utils_test ${PROJECT_NAME})

endif()

ament_package()
