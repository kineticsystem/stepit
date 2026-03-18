# Add support for GTest.
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(ros2_control_test_assets REQUIRED)
find_package(cobs_serial REQUIRED)

ament_lint_auto_find_test_dependencies()

# GMock throws an error if we don't switch off this option in tests.
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wno-sign-conversion)
endif()

###############################################################################
# test_stepit_hardware

ament_add_gmock(test_stepit_hardware
    test_stepit_hardware.cpp
    mock/mock_driver.hpp
    mock/mock_driver_factory.hpp
    fake/fake_hardware_info.hpp
)
target_include_directories(test_stepit_hardware
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_stepit_hardware
    stepit_hardware
)
ament_target_dependencies(test_stepit_hardware
    cobs_serial
    pluginlib
    ros2_control_test_assets
)
