# Add support for GTest.
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(ros2_control_test_assets REQUIRED)

ament_lint_auto_find_test_dependencies()

# GMock throws an error if we don't switch off this option in tests.
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wno-sign-conversion)
endif()

###############################################################################
# test_data_utils

ament_add_gmock(test_data_utils test_data_utils.cpp)
target_link_libraries(test_data_utils cobs_serial)

###############################################################################
# test_crc_utils

ament_add_gmock(test_crc_utils test_crc_utils.cpp)
target_link_libraries(test_crc_utils cobs_serial)

###############################################################################
# test_default_data_interface

ament_add_gmock(test_default_cobs_serial
    test_default_cobs_serial.cpp
    mock/mock_serial.hpp
)
target_include_directories(test_default_cobs_serial
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_default_cobs_serial cobs_serial)
