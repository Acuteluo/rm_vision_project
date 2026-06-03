clear

printf "\033[1;33m>>> 开始构建 <<<\033[0m\n"

colcon build

printf "\033[1;33m>>> 刷新环境变量 <<<\033[0m\n"

. install/setup.bash

export LD_LIBRARY_PATH=/home/cly/.local/lib/python3.10/site-packages/openvino/libs:$LD_LIBRARY_PATH

printf "\033[1;33m>>> 启动 launch 文件，日志写入  my_logger/0603 <<<\033[0m\n"

ros2 launch bringup offline_video_launch.py 2>&1 | tee my_logger/0603.txt
