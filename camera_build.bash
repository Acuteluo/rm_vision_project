clear

printf "\033[1;33m>>> 开始构建 <<<\033[0m\n"

colcon build

printf "\033[1;33m>>> 刷新环境变量 <<<\033[0m\n"

. install/setup.bash

# 必须先 source OpenVINO 的环境，否则会报找不到 libopenvino.so
source ~/openvino_env/setupvars.sh

printf "\033[1;33m>>> 启动 launch 文件，日志写入  my_logger/0610_camera <<<\033[0m\n"

ros2 launch bringup total_launch_galaxy.py 2>&1 | tee my_logger/0610_camera.txt
