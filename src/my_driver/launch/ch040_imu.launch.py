##launch file
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config={"serial_port":"/dev/ch040_imu",
            # "baud_rate":115200,
            "baud_rate":921600,
            "frame_id":"imu_link",
            "imu_topic":"/ch040imu/data",
            }
    return LaunchDescription([
         Node(
            package='hipnuc_imu',
            executable='talker',
            name='IMU_publisher',
            parameters=[config],
            output='screen',
            )
        ])

