cmake_minimum_required(VERSION 3.10)
project(my_mindvision_camera) # project 是包名

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 添加库（如果源码只有一个文件，直接添加）
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/mv_camera_node.cpp   # 注意文件名
)

# 添加 SDK 头文件路径
target_include_directories(${PROJECT_NAME} PUBLIC mvsdk/include)

# 根据架构链接 SDK 库
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC mvsdk/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(${PROJECT_NAME} PUBLIC mvsdk/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture!")
endif()

target_link_libraries(${PROJECT_NAME} MVSDK)

# 注册组件，生成可执行文件（可自定义名字）
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "mindvision_camera::MVCameraNode"   # 注意 命名空间 需与代码一致（不改，就用他的命名空间 mindvision_camera）
  EXECUTABLE mv_camera_node # 可执行文件名称，影响 launch
)

# 如果测试，可保留，否则可删除

# 安装 config 和 launch 目录
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)