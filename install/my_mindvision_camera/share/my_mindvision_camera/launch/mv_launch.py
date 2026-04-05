import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取配置文件路径
    params_file = os.path.join(
        get_package_share_directory('my_mindvision_camera'), 'config', 'camera_params.yaml')

    camera_info_url = 'package://my_mindvision_camera/config/camera_info.yaml' # 注意 package:// 后面是包名

    return LaunchDescription([
        # 原有参数
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        DeclareLaunchArgument(name='camera_info_url',
                              default_value=camera_info_url),
        DeclareLaunchArgument(name='use_sensor_data_qos',
                              default_value='false'),

        # ===== 新增相机参数（可命令行覆盖）=====
        DeclareLaunchArgument('exposure_time', default_value='2000', # 原始是 5000
                              description='Exposure time in microseconds'),
        DeclareLaunchArgument('analog_gain', default_value='5',
                              description='Analog gain'),
        DeclareLaunchArgument('rgb_gain_r', default_value='100',
                              description='Red gain'),
        DeclareLaunchArgument('rgb_gain_g', default_value='100',
                              description='Green gain'),
        DeclareLaunchArgument('rgb_gain_b', default_value='100',
                              description='Blue gain'),
        DeclareLaunchArgument('saturation', default_value='100',
                              description='Saturation'),
        DeclareLaunchArgument('gamma', default_value='200', # 原始是 100
                              description='Gamma'),
        DeclareLaunchArgument('flip_image', default_value='false',
                              description='Flip image'),

        Node(
            package='my_mindvision_camera',
            executable='mv_camera_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('params_file'),  # 先加载 YAML 文件
                {
                    'camera_info_url': LaunchConfiguration('camera_info_url'),
                    'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
                    # 新增参数映射（键名与相机节点 declare_parameter 一致）
                    'exposure_time': LaunchConfiguration('exposure_time'),
                    'analog_gain': LaunchConfiguration('analog_gain'),
                    'rgb_gain.r': LaunchConfiguration('rgb_gain_r'),
                    'rgb_gain.g': LaunchConfiguration('rgb_gain_g'),
                    'rgb_gain.b': LaunchConfiguration('rgb_gain_b'),
                    'saturation': LaunchConfiguration('saturation'),
                    'gamma': LaunchConfiguration('gamma'),
                    'flip_image': LaunchConfiguration('flip_image'),
                }
            ],
        )
    ])