cmake_minimum_required(VERSION 3.10)
project(img_processing)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(serial_driver_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# 强制使用我们下载的独立版 OpenVINO
set(OpenVINO_DIR "/home/cly/openvino_cpp/runtime/cmake")
find_package(OpenVINO REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(core_node
    src/core_node.cpp
    src/tf.cpp
    src/ekf.cpp
    src/plotter.cpp
    src/yolo_detector.cpp  
    src/yolo_armor.cpp     
)

target_include_directories(core_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(core_node
    rclcpp sensor_msgs cv_bridge serial_driver_interfaces tf2_ros tf2_geometry_msgs geometry_msgs
)

target_link_libraries(core_node
    ${OpenCV_LIBS}
    Eigen3::Eigen
    openvino::runtime  
)

# 【终极护盾】：把库的路径死死烧录进程序，防止被系统干扰
set_target_properties(core_node PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "/home/cly/openvino_cpp/runtime/lib/intel64;/home/cly/openvino_cpp/runtime/3rdparty/tbb/lib"
)

install(TARGETS core_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY model DESTINATION share/${PROJECT_NAME})
ament_package()