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
    ld.add_action(DeclareLaunchArgument('use_fast_lio_tf', default_value='true', description='增加point lio修正'))
    ld.add_action(DeclareLaunchArgument('use_livox_publish', default_value='true', description='增加雷达到车体中心的tf发布'))  
    ld.add_action(DeclareLaunchArgument('use_tf_publish',default_value='true',description='Publish tf tree if use is True'))
    ld.add_action(DeclareLaunchArgument('use_realsense',default_value='true',description='Start realsense node if use is True'))
    ld.add_action(DeclareLaunchArgument('record_lidar',default_value='false',description='Record lidar data if use is True'))
    ld.add_action(DeclareLaunchArgument('record_imu',default_value='true',description='Record imu data if use is True'))
    ld.add_action(DeclareLaunchArgument('record_images',default_value='false',description='Record images if use is True'))
    ld.add_action(DeclareLaunchArgument('record_nav',default_value='true',description='Record nav data if use is True'))
    get_package_share_directory('my_driver')
    get_package_share_directory('rc_bringup')
    #启动mid360
    mid360_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'),'launch','mid360_bringup.launch.py')
        ),
        launch_arguments={
            'use_rviz': 'false',  #不启动rviz
        }.items(),
    )
    #启动imu转换
    imu_transform_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('perception'),'launch','imu_transform.launch.py')
        ),
    )
    #启动utils
    utils_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rc_bringup'),'launch','utils_bringup.launch.py')
        ),
        launch_arguments={
            # "xacro_file": os.path.join(get_package_share_directory('my_tf_tree'),'urdf','laser_base.xacro'),
            "xacro_file": os.path.join(get_package_share_directory('my_tf_tree'),'urdf','r2.urdf.xacro'),
        }.items()
    )
    #启动realsense
    realsense_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'),'launch','realsense_bringup.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_realsense'))
    )
    #启动手柄
    joy_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'),'launch','joy.launch.py')
        ),
    )
    #启动下位机通信
    communicate_node=Node(
        package='my_driver',
        executable='com.py',
        name='communicate',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'serial_port': '/dev/serial_ch340',
             'serial_baudrate':230400,
             }
        ]
    )
    sick_node=Node(
        package='my_driver',
        executable='sick_com.py',
        name='sick_node',
        output='screen',
        emulate_tty=True,
    )
    #启动ms200
    ms200_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'),'launch','ms200_scan.launch.py')
        ),
    )
    ros_bag_node=  Node(
                    # condition=IfCondition(LaunchConfiguration('use_rosbag_record')),
                    package='python_pkg',
                    executable='rosbag_record',
                    name='rosbag_record',
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        {'record_lidar': LaunchConfiguration('record_lidar')},
                        {'record_imu': LaunchConfiguration('record_imu')},
                        {'record_images': LaunchConfiguration('record_images')},
                        {'record_nav': LaunchConfiguration('record_nav')}
                    ]
                )
    ros_bag_action=TimerAction(
        period=5.0,  # Delay in seconds
        actions=[ros_bag_node]
    )
    fusion_node=Node(
        package='perception',
        executable='fusion.py',
        name='fusion_node',
        output='screen',
        parameters=[

           { 'use_sick': False},  # 使用点云数据

        ])
    #再开启新的xacro发布
    xacro_file_path=PathJoinSubstitution(
        [get_package_share_directory('my_tf_tree'), 'urdf', 'filter_base.xacro']
    )
    robot_description = Command([
        FindExecutable(name='xacro'),  # 查找 xacro 可执行文件
        ' ',
        xacro_file_path,  # 使用 xacro 文件路径
    ])
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )
    odomtransform_tf_node = Node(
        condition=IfCond'''
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
    ld.add_action(DeclareLaunchArgument('use_extern_imu',default_value='false',description='Start extern imu node if use is True'))
    ld.add_action(DeclareLaunchArgument('use_imu_transform',default_value='true',description='Start imu transform node if use is True'))
    ld.add_action(DeclareLaunchArgument('record_lidar',default_value='true',description='Record lidar data if use is True'))
    ld.add_action(DeclareLaunchArgument('record_imu',default_value='true',description='Record imu data if use is True'))
    ld.add_action(DeclareLaunchArgument('record_images',default_value='false',description='Record images if use is True'))
    ld.add_action(DeclareLaunchArgument('record_nav',default_value='true',description='Record nav data if use is True'))
    get_package_share_directory('my_driver')
    get_package_share_directory('rc_bringup')
    #启动mid360
    mid360_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'),'launch','mid360_dd.launch.py')
        ),
        launch_arguments={
            'use_rviz': 'false',  #不启动rviz
        }.items(),
    )
    #启动imu转换
    imu_transform_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('perception'),'launch','imu_transform.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_imu_transform'))
    )
    #启动utils
    utils_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rc_bringup'),'launch','utils_bringup.launch.py')
        ),
    )
    #启动手柄
    joy_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'),'launch','joy.launch.py')
        ),
    )
    #启动下位机通信
    communicate_node=Node(
        package='my_driver',
        executable='com.py',
        name='communicate',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'serial_port': '/dev/serial_qh',
             'serial_baudrate':230400,
             }
        ]
    )
    #启动ms200
    ms200_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'),'launch','ms200_scan.launch.py')
        ),
    )
    ros_bag_node=  Node(
                    # condition=IfCondition(LaunchConfiguration('use_rosbag_record')),
                    package='python_pkg',
                    executable='rosbag_record',
                    name='rosbag_record',
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        {'record_lidar': LaunchConfiguration('record_lidar')},
                        {'record_imu': LaunchConfiguration('record_imu')},
                        {'record_images': LaunchConfiguration('record_images')},
                        {'record_nav': LaunchConfiguration('record_nav')}
                    ]
                )
    ros_bag_action=TimerAction(
        period=5.0,  # Delay in seconds
        actions=[ros_bag_node]
    )
    fusion_node=Node(
        package='perception',
        executable='fusion.py',
        name='fusion_node',
        output='screen',
        parameters=[
            {'lidar_x_bias': 0.23751}, #odom到激光雷达的偏移,odom是子坐标系，激光雷达是父坐标系
            {'lidar_y_bias': 0.24275},
           { 'use_sick': False},  # 使用点云数据
            {'slam_debug': False},  # 是否开启slam调试
        ]) # 向
    xacro_file_path:str= os.path.join(get_package_share_directory('my_tf_tree'),'urdf','dd.urdf.xacro')
    robot_description = Command([
        FindExecutable(name='xacro'),  # 查找 xacro 可执行文件
        ' ',  # 确保命令和路径之间有空格
        xacro_file_path  # 传入 xacro 文件路径
    ])

    # 机器人状态发布节点
    robot_state_publisher_node = ComposableNode(
        package='robot_state_publisher',
        plugin='robot_state_publisher::RobotStatePublisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )
    odom_to_transform = ComposableNode(
        package='tf2_ros',
        plugin='tf2_ros::StaticTransformBroadcasterNode',
        name='tf_broadcaster1',
        parameters=[{
            'child_frame_id': 'odom_transform',  # 旋转后坐标系
            'frame_id': 'odom',  # 参考坐标系
            'translation': {'x': 0.23125+0.23751+0.2958, 'y': 7.1525, 'z': 0.0}, 
            'rotation': {'x':0.0, 'y':0.0, 'z':0.0, 'w':1.0}  # 四元数表示的 90 度旋转（绕 Z 轴）
        }],
    )
    map_to_camera = ComposableNode(
        package='tf2_ros',
        plugin='tf2_ros::StaticTransformBroadcasterNode',
        name='tf_broadcaster1',
        parameters=[{
            'child_frame_id': 'camera_init',  # 旋转后坐标系
            'frame_id': 'map',  # 参考坐标系
            'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 
            'rotation': {'x':0.0, 'y':0.0, 'z':0.7071, 'w':-0.7071}  # 四元数表示的 90 度旋转（绕 Z 轴）
        }],
    )
    map_to_odom_tf = ComposableNode(
        package='tf2_ros',
        plugin='tf2_ros::StaticTransformBroadcasterNode',
        name='map_to_odom_tf_node',
        parameters=[{
            'child_frame_id': 'odom',  # 旋转后坐标系
            'frame_id': 'map',  # 参考坐标系
            'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 
            'rotation': {'x':0.0, 'y':0.0, 'z':0.0, 'w':1.0}  # 四元数表示的 90 度旋转（绕 Z 轴）
        }],
    )
    compose_node=ComposableNodeContainer(
        namespace='',
        name='start_container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[map_to_odom_tf, odom_to_transform, map_to_camera,robot_state_publisher_node],
        arguments=['--ros-args', '--log-level', 'fatal'],
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(compose_node)
    ld.add_action(fusion_node)
    ld.add_action(mid360_launch)
    ld.add_action(imu_transform_launch)
    ld.add_action(utils_launch)
    ld.add_action(joy_launch)
    ld.add_action(communicate_node)
    # ld.add_action(ms200_launch)
    ld.add_action(ros_bag_action)
    return ld
     

    # ld.add_action(robot_state_publisher_node)
    ld.add_action(sick_node)
    ld.add_action(fusion_node)
    ld.add_action(mid360_launch)
    ld.add_action(imu_transform_launch)
    ld.add_action(realsense_launch)
    ld.add_action(utils_launch)
    ld.add_action(joy_launch)
    ld.add_action(communicate_node)
    ld.add_action(ms200_launch)
    ld.add_action(ros_bag_action)
    return ld
     