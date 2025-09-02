'''
启动驱动节点和工具节点
'''
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess,IncludeLaunchDescription,DeclareLaunchArgument,TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node 
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
def generate_launch_description():
    ld=LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_tf_publish',default_value='false',description='Publish tf tree if use is True'))
    ld.add_action(DeclareLaunchArgument('use_ros1_bridge',default_value='false',description='Use ros1_bridge if use is True'))
    fox_glove=Node(
        package='foxglove_bridge',
        executable='foxglove_bridge'
    )
    joy_driver = Node(
        package='joy',
        executable='joy_node',
        name='joy_driver_node')
    bridge_node=Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        output='screen'
    )
    ld.add_action(joy_driver)
    ld.add_action(bridge_node)
    ld.add_action(fox_glove)
    return ld