from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rm_serial_driver',
            executable='rm_serial_driver_node',
            name='serial_driver',
            parameters=[{
                'device_name': '/dev/ttyACM0',
                'baud_rate': 115200,
                'flow_control': 'none',
                'parity': 'none',
                'stop_bits': '1'
            }],
            output='screen'
        )
    ])