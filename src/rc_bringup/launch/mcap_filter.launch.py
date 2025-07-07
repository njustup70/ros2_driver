from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='python_pkg',
            executable='mcap_filter',
            name='filter_mcap_node',
            output='screen',
            parameters=[
                # {'rosbag_root_path': '/data'},
                {'whitelist': ['/livox/lidar/pc']},
                {'start_time': '2025-07-07T00:00:00Z'},
                {'end_time': '2025-07-07T02:00:00Z'}
            ]
        )
    ])
