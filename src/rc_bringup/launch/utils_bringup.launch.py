'''
启动rosbag 记录数据 
rosbridge 桥接ros1 ros2话题
启动foxglove用来可视化
'''
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess,IncludeLaunchDescription,DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
def generate_launch_description():
    local_path=os.path.join(get_package_share_directory('rc_bringup'))
    ld=LaunchDescription()
    
    ld.add_action(DeclareLaunchArgument('use_tf_publish',default_value='false',description='Publish tf tree if use is True'))
    ld.add_action(DeclareLaunchArgument('use_ros1_bridge',default_value='true',description='Use ros1_bridge if use is True'))
    ld.add_action(DeclareLaunchArgument('use_fast_lio_tf',default_value='false',description='提供fast_lio的tf树'))
    ld.add_action(DeclareLaunchArgument('xacro_file',default_value=os.path.join(get_package_share_directory('my_tf_tree'),'urdf','r2.urdf.xacro'),description='xacro file path'))
    ld.add_action(DeclareLaunchArgument('use_rosbridge',default_value='true',description='是否开启websocket桥接'))
    # ld.add_action(DeclareLaunchArgument('ros', default_value='5', description='Max number of rosbag files'))
    foxglove_node=ComposableNode(
        package='foxglove_bridge',
        plugin='foxglove_bridge::FoxgloveBridge',
        name='foxglove_bridge_node',
        parameters=[ {'send_buffer_limit': 1000000000}],
        extra_arguments=[{'use_intra_process_comms': True},
                    {'use_multi_threaded_executor': True},
                    {'ros__arguments': ['--log-level', 'fatal']}
        ]
        )
    ros_master_exe=ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('use_ros1_bridge')),
        cmd=["bash","-c","cd ~/ros2_driver/packages/roscore && sudo docker-compose up"]
    )
    ros_bridge_exe=ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('use_ros1_bridge')),
        # cmd=["bash","-c","~/docker/ros2-modules/packages/ros-bridge/ros_bridge_run.sh"],
        cmd=["bash","-c","python3 ~/ros2_driver/packages/ros-bridge/ros_bridge_run.py"],
        output='screen',
    )
    # ros_bag_bash_path=os.path.join(local_path,'scripts/rosbag_record.py')
    #需要condition来判断是否启动rosbag
    # ros_bag_exe=ExecuteProcess(
    #     condition=IfCondition(LaunchConfiguration('use_rosbag_record')),
    #     # cmd=["bash","-c","sleep 5 && python3 {}".format(ros_bag_bash_path)],
    #     output='screen',
    #     emulate_tty=True,
    # )
    
    #TF树相关
    xacro_file_path=LaunchConfiguration('xacro_file') # 获取 xacro 文件路径
    # 解析 Xacro 文件并生成 URDF
    robot_description = Command([
        FindExecutable(name='xacro'),  # 查找 xacro 可执行文件
        ' ',  # 确保命令和路径之间有空格
        xacro_file_path  # 传入 xacro 文件路径
    ])

    # 机器人状态发布节点
    robot_state_publisher_node = ComposableNode(
        condition=IfCondition(LaunchConfiguration('use_tf_publish')),
        package='robot_state_publisher',
        plugin='robot_state_publisher::RobotStatePublisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )
    # fast lio tf支持
    fast_lio_tf_node=ComposableNode(
        condition=IfCondition(LaunchConfiguration('use_fast_lio_tf')),
        package='tf2_ros',
        plugin='tf2_ros::StaticTransformBroadcasterNode',
        name='tf_broadcaster',
        parameters=[{
        'child_frame_id': 'base_link',
        'frame_id': 'body',
        'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 
        'rotation': [0.0, 0.0, 0.0]
    }],
    )
    fast_lio_tf_node2 = ComposableNode(
        condition=IfCondition(LaunchConfiguration('use_fast_lio_tf')),
        package='tf2_ros',
        plugin='tf2_ros::StaticTransformBroadcasterNode',
        name='tf_broadcaster1',
        parameters=[{
            'child_frame_id': 'camera_init',  # 旋转后坐标系
            'frame_id': 'odom',  # 参考坐标系
            'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 
            'rotation': {'x':0.0, 'y':0.0, 'z':0.7071, 'w':-0.7071}  # 四元数表示的 90 度旋转（绕 Z 轴）
        }],
    )
    map_to_odom_tf_node = ComposableNode(
        condition=IfCondition(LaunchConfiguration('use_tf_publish')),
        package='tf2_ros',
        plugin='tf2_ros::StaticTransformBroadcasterNode',
        name='map_to_odom_tf_node',
        parameters=[{
            'child_frame_id': 'odom',  # 旋转后坐标系
            'frame_id': 'map',  # 参考坐标系
            'translation': {'x': 1.0, 'y': 1.0, 'z': 0.0}, 
            'rotation': {'x':0.0, 'y':0.0, 'z':0.0, 'w':1.0}  # 四元数表示的 90 度旋转（绕 Z 轴）
        }],
    )


    compose_container=ComposableNodeContainer(
        namespace='',
        name='start_container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[foxglove_node,robot_state_publisher_node,fast_lio_tf_node,fast_lio_tf_node2,map_to_odom_tf_node],
        arguments=['--ros-args', '--log-level', 'fatal'],
        output='screen',
        emulate_tty=True,
    )
    # websocket_bridge=IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('rosbridge_server'),'launch','rosbridge_websocket_launch.xml')
    #     ),
    #     condition=IfCondition(LaunchConfiguration('use_rosbridge')),
    # )
    websocket_bridge=ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('use_rosbridge')),
        cmd=["bash","-c","ros2 launch rosbridge_server rosbridge_websocket_launch.xml"],
       
    )
    ld.add_action(ros_master_exe)
    ld.add_action(ros_bridge_exe)
    # ld.add_action(ros_bag_exe)
    # ld.add_action(ros_bag_action)
    ld.add_action(compose_container)
    ld.add_action(websocket_bridge)
    return ld
