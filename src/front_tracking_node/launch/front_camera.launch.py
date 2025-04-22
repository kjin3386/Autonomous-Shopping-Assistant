from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            name='PYTHONPATH',
            value='/home/incsl-wego22/ws_capstone/src/front_tracking_node/3rdparty/OC_SORT/trackers'
        ),
        Node(
            package='front_tracking_node',
            executable='front_camera',
            name='front_camera',
            output='screen'
        )
    ])

