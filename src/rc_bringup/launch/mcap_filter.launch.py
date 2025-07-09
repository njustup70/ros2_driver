from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='python_pkg',
            executable='mcap_filter',
            name='filter_mcap_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                # {'rosbag_root_path': '/data'},
                {'whitelist': ['/livox/lidar/pc','/livox/imu/normal','/odom','livox/imu']},
                {'start_time': ''},
                {'end_time': ''}
            ]
            
        )
    ])
