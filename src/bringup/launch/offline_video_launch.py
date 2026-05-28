from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    core_node = Node(
            package='img_processing',      
            executable='core_node',    
            name='core_node_offline',
            output='screen',
            parameters=[{
                'core.mode.is_video_mode': True,

                # 视频路径
                'core.mode.video_path': '/home/cly/下载/rm_test_videos/20260501_160636__camera_0_rgb_output.mp4',
                
                'is_standalone_mode': True, # 单机模式（不依赖串口），视频模式下必须为 True
            }]
        )
    return LaunchDescription([core_node])