# Add support for GTest.
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)

ament_add_gmock(data_utils_test data_utils_test.cpp)
target_include_directories(data_utils_test
  PRIVATE {CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_options(data_utils_test
  PRIVATE -Wall -Wextra
)
target_link_libraries(data_utils_test ${PROJECT_NAME})

ament_add_gmock(crc_utils_test crc_utils_test.cpp)
target_include_directories(crc_utils_test
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_options(crc_utils_test
  PRIVATE -Wall -Wextra
)
target_link_libraries(crc_utils_test ${PROJECT_NAME})

ament_add_gmock(data_interface_test data_interface_test.cpp
    mock_serial.hpp
)
target_include_directories(data_interface_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_options(data_interface_test
  PRIVATE -Wall -Wno-return-type
)
target_link_libraries(data_interface_test ${PROJECT_NAME})
