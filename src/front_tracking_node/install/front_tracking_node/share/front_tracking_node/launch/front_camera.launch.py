from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='front_tracking_node',
            executable='front_camera',
            name='front_tracking_node',
            output='screen'
        )
    ])

