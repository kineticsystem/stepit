# Add support for GTest.
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ros2_control_test_assets REQUIRED)

# GMock throws an error if we don't switch off this option in tests.
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wno-sign-conversion)
endif()

###############################################################################
# test_data_utils

ament_add_gmock(test_data_utils test_data_utils.cpp)
target_include_directories(test_data_utils
  PRIVATE {CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_data_utils stepit_hardware)

###############################################################################
# test_crc_utils

ament_add_gmock(test_crc_utils test_crc_utils.cpp)
target_include_directories(test_crc_utils
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_crc_utils stepit_hardware)

###############################################################################
# test_data_interface

ament_add_gmock(test_data_interface test_data_interface.cpp)
target_include_directories(test_data_interface
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_data_interface stepit_hardware)

###############################################################################
# test_load_stepit_hardware

ament_add_gmock(test_stepit_hardware test_stepit_hardware.cpp)
target_include_directories(test_stepit_hardware
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_stepit_hardware
    stepit_hardware

)
ament_target_dependencies(test_stepit_hardware
  pluginlib
  ros2_control_test_assets
)
