cmake_minimum_required(VERSION 3.8)
project(img_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(message_filters REQUIRED)

find_package(serial_driver_interfaces REQUIRED)

# 本包的头文件目录
include_directories(include)


# 节点
add_executable(core_node src/core_node.cpp src/prepare_algorithm.cpp src/strip.cpp src/armorplate.cpp src/ekf.cpp src/kf_data.cpp src/kf_position.cpp src/tf.cpp src/plotter.cpp)


target_include_directories(core_node PRIVATE
  include
)

ament_target_dependencies(core_node rclcpp sensor_msgs cv_bridge OpenCV Eigen3 message_filters serial_driver_interfaces tf2_ros tf2_geometry_msgs geometry_msgs)

target_link_libraries(core_node
  ${OpenCV_LIBS}
)





# 安装可执行文件
install(TARGETS core_node DESTINATION lib/${PROJECT_NAME})

# 安装本包的头文件
install(DIRECTORY include/ DESTINATION include)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()