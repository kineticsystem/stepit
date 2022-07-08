# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.16.3)
project(stepit_hardware LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)

# Build stepit_hardware library.

add_library(${PROJECT_NAME} # QCreator needs header files to display them.
  src/request.cpp
  include/stepit_hardware/request.h
  src/response.cpp
  include/stepit_hardware/response.h
  src/motor_move_to_command.cpp
  include/stepit_hardware/motor_move_to_command.h
  src/motor_status.cpp
  include/stepit_hardware/motor_status.h
  src/data_utils.cpp
  include/stepit_hardware/data_utils.h
  include/stepit_hardware/buffer.h
  include/stepit_hardware/buffer_position.h
  src/data_buffer.cpp
  include/stepit_hardware/data_buffer.h
  src/command_interface.cpp
  include/stepit_hardware/command_interface.h
  include/stepit_hardware/serial_interface.h
  src/crc_utils.cpp
  include/stepit_hardware/crc_utils.h
  src/serial_interface_impl.cpp
  include/stepit_hardware/serial_interface_impl.h
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
  PRIVATE cxx_std_20
)

add_executable(exec
  src/main.cpp
  src/data_utils.cpp
  include/stepit_hardware/data_utils.h
  src/serial_interface_impl.cpp
  include/stepit_hardware/serial_interface_impl.h
)
target_include_directories(exec
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>)
ament_target_dependencies(exec
  rclcpp
  serial
)
target_compile_options(${PROJECT_NAME}
  PRIVATE -Wall -Wextra -Wpedantic
)
target_compile_features(${PROJECT_NAME}
  PRIVATE cxx_std_20
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

    # Add test.
    ament_add_gtest(crc_utils_test test/crc_utils_test.cpp)
    target_include_directories(crc_utils_test
      PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      PUBLIC $<INSTALL_INTERFACE:include>)
    target_compile_options(crc_utils_test
      PRIVATE -Wall -Wextra -Wpedantic
    )
    target_link_libraries(crc_utils_test ${PROJECT_NAME})

endif()

ament_package()
