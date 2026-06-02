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
                
                'is_standalone_mode': True, # 单机模式（不依赖串口），视频模式下必须为 True
            }]
        )
    
    # # 添加 Python 推理节点
    # yolo_infer_node = Node(
    # package='img_processing',
    # executable='yolo_infer_node.py',
    # name='yolo_infer_node',
    # output='screen',
    # # 如果这个节点需要订阅图像话题，确保话题名正确
    # )
 
    return LaunchDescription([core_node])