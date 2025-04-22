from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rear_tracking_node',
            executable='rear_camera',
            name='rear_tracking_node',
            output='screen'
        )
    ])

