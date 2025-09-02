from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
def generate_launch_description():
    ld=LaunchDescription()
    ld.add_action(DeclareLaunchArgument('rviz', default_value='true', description='Start rviz if use is True'))
    my_pacakge_path = get_package_share_directory('my_driver')
    rviz_config=get_package_share_directory('rslidar_sdk')+'/rviz/rviz2.rviz'
    rviz_node= Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    config_file = os.path.join(my_pacakge_path, 'config', 'rs_airy.yaml')
    rslidar_sdk_node = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        output='screen',
        parameters=[{'config_path': config_file}],
        namespace='rslidar_sdk'
    )
    ld.add_action(rslidar_sdk_node)
    ld.add_action(rviz_node)
    return ld