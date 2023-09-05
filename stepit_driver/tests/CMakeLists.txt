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
# test_default_driver

ament_add_gmock(test_default_driver
    test_default_driver.cpp
)
target_include_directories(test_default_driver
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_default_driver stepit_driver)

###############################################################################
# test_fake_motor

ament_add_gmock(test_fake_motor
    test_fake_motor.cpp
)
target_include_directories(test_fake_motor
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_fake_motor stepit_driver)

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
stepit_driver
)
ament_target_dependencies(test_stepit_hardware
    pluginlib
    ros2_control_test_assets
)
