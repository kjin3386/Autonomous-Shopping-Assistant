from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_path = os.path.join(
        os.getenv('HOME'),
        'ws_capstone',
        'src',
        'slam_launch',
        'config',
        'mapper_params_localization.yaml'
    )

    return LaunchDescription([

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'rplidar',
                'inverted': False,
                'angle_compensate': True
            }],
            output='screen'
        ),

        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                config_path,
                {'use_sim_time': False}
            ],
            remappings=[
                ('/odom', '/scout_mini_base_controller/odom')
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rplidar_tf',
            arguments=['0', '0', '0.9', '0', '0', '1', '0', 'base_link', 'rplidar'],
            output='screen'
        )
    ])

