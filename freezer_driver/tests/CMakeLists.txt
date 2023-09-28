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
# test_freezer_hardware

ament_add_gmock(test_freezer_hardware
    test_freezer_hardware.cpp
)
target_include_directories(test_freezer_hardware
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_freezer_hardware
    freezer_driver
)
ament_target_dependencies(test_freezer_hardware
    pluginlib
    ros2_control_test_assets
)

###############################################################################
# test_default_driver_factory

ament_add_gmock(test_default_driver_factory
    test_default_driver_factory.cpp
    mock/mock_driver.hpp
)
target_include_directories(test_default_driver_factory
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_default_driver_factory freezer_driver)
