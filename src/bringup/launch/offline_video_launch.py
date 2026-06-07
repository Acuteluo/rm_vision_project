from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    path1 = '/home/cly/下载/rm_test_videos/20260501_193831__camera_0_rgb_output.mp4' # 移动中速靶
    path2 = '/home/cly/下载/rm_test_videos/20260501_160945__camera_0_output.mp4'     # 静止高速靶
    path3 = '/home/cly/下载/rm_test_videos/20260501_160636__camera_0_rgb_output.mp4' # 静止中低速变速靶（常用）
    path4 = '/home/cly/下载/rm_test_videos/972e9fe56e87e60e104c6042599bae5c.mp4'     # 实录静止远距离靶
    path5 = '/home/cly/下载/rm_test_videos/armorplate.mp4'                           # 第三轮考核视频

    core_node = Node(
        package='img_processing',      
        executable='core_node',    
        name='core_node_offline',
        output='screen',
        parameters=[{
            'core.mode.is_video_mode': True, # 启用本地视频模式
                
            'core.mode.video_path': path1,

            'is_standalone_mode': True, # 单机模式（不依赖串口），视频模式下必须为 True
        }]
    )
 
    return LaunchDescription([core_node])