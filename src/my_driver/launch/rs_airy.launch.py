from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    my_pacakge_path = get_package_share_directory('my_driver')

    rviz_config=get_package_share_directory('rslidar_sdk')+'/rviz/rviz2.rviz'
    
    config_file = os.path.join(my_pacakge_path, 'config', 'rs_airy.yaml')
    
    return LaunchDescription([
        Node(namespace='rslidar_sdk', package='rslidar_sdk', executable='rslidar_sdk_node', output='screen', parameters=[{'config_path': config_file}]),
        Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config])
    ])