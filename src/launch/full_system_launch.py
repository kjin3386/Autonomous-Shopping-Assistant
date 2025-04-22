from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='front_camera_node',
            executable='front_camera_node',
            name='front_camera_node',
            output='screen'
        ),
        Node(
            package='rear_tracking_node',
            executable='rear_tracking_node',
            name='rear_tracking_node',
            output='screen'
        ),
        Node(
            package='user_follow_decision_node',
            executable='user_decision_node',
            name='user_decision_node',
            output='screen'
        ),
        Node(
            package='path_planning_node',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen'
        )
    ])

