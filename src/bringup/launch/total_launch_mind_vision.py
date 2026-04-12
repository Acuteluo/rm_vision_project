from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess  # cmd指令封装成这个类的对象
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
# from launch.actions import DeclareLaunchArgument  # 声明键值对
# from launch.substitutions import LaunchConfiguration  # 通过键解析值
# 文件包含相关-------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 被包含的 launch 文件路径封装成这个类的对象
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace  # 设置命名空间
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit  # 事件对象，节点启动时 / 节点关闭时
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo  # 注册事件 打印日志
# 获取功能包下share目录路径-------
# os.path.join 拼装路径, get_package_share_directory(包名)可以获取到share下面这个包名文件夹的路径，再拼接上 config 和 .yaml即可
# from ament_index_python.packages import get_package_share_directory
# import os

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    camera = Node(
        package="my_mindvision_camera", 
        executable="mv_camera_node", 
        name="mind_vision_camera",
        # parameters=[
        # {'exposure_time': 500}, # very good
        # {'analog_gain': 5},
        # {'rgb_gain.r': 100},
        # {'rgb_gain.g': 100},
        # {'rgb_gain.b': 100},
        # {'saturation': 100},
        # {'gamma': 200},
        # {'flip_image': False}]
        parameters=[
        {'exposure_time': 1000}, # 原本5000，1000效果好
        {'analog_gain': 5},
        {'rgb_gain.r': 100},
        {'rgb_gain.g': 100},
        {'rgb_gain.b': 100},
        {'saturation': 100},
        {'gamma': 200},
        {'flip_image': False}]
    )

    core_node = Node(
        package="img_processing", 
        executable="core_node", 
        name="core_node",
        output='screen'
    )

    tf_node = Node(
        package='img_processing',
        executable='tf_node',
        name='tf_node',
        output='screen'
    )

    serial_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rm_serial_driver'), '/launch', '/serial_driver.launch.py'
        ])
    )

    return LaunchDescription([camera, core_node, tf_node, serial_driver_launch])