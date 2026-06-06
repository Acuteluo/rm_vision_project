cmake_minimum_required(VERSION 3.10)
project(unit_tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(GTest REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${GTEST_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../rm_serial_driver/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../img_processing/include
)

# CRC unit tests
add_executable(test_crc
    src/test_crc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../rm_serial_driver/src/crc.cpp
)
target_link_libraries(test_crc ${GTEST_BOTH_LIBRARIES} pthread)

# Packet serialization unit tests
add_executable(test_packet
    src/test_packet.cpp
)
target_link_libraries(test_packet ${GTEST_BOTH_LIBRARIES} pthread)

# Strip geometry unit tests (tests the core algorithms without ROS2 dependency)
add_executable(test_strip
    src/test_strip.cpp
)
target_link_libraries(test_strip ${GTEST_BOTH_LIBRARIES} ${OpenCV_LIBRARIES} Eigen3::Eigen pthread)

# Combined test runner
add_custom_target(run_all_tests
    COMMAND test_crc
    COMMAND test_packet
    COMMAND test_strip
    DEPENDS test_crc test_packet test_strip
    COMMENT "Running all unit tests"
)
