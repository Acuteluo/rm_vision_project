cmake_minimum_required(VERSION 3.10)
project(img_processing)

# 1. 编译标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall -Wextra)

# 2. 查找 ROS2 依赖 (【核心修改】：已经彻底删除了 cv_bridge)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial_driver_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

# ===== OpenVINO GPU 测试开关 =====
# 注释掉下面这行，就会退回到 CPU 版 OpenCV
set(OpenCV_DIR "/home/cly/opencv_openvino/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)

# 4. 头文件路径
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# 5. 可执行文件
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

# 6. 链接 ROS2 依赖 (【核心修改】：去掉了 cv_bridge)
ament_target_dependencies(core_node
    rclcpp 
    sensor_msgs 
    serial_driver_interfaces 
    tf2_ros 
    tf2_geometry_msgs 
    geometry_msgs
)

# 7. 链接第三方库
target_link_libraries(core_node
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
)

# 8. 安装规则
install(TARGETS core_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY model DESTINATION share/${PROJECT_NAME})

ament_package()