import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
def generate_launch_description():
    my_path=get_package_share_directory('perception')
    param_file_path=os.path.join(my_path,'config','Initialization_result.txt')
    ld=LaunchDescription()
    ld.add_action(DeclareLaunchArgument("imu_topic",default_value="/imu"))
    ld.add_action(DeclareLaunchArgument("use_grivaty2m",default_value="true"))
    ld.add_action(DeclareLaunchArgument("imu_transformed_topic",default_value="/imu_transformed"))
    ld.add_action(DeclareLaunchArgument("imu_frame",default_value="imu_link"))
    ld.add_action(DeclareLaunchArgument("lidar_frame",default_value="livox_frame"))
    ld.add_action(DeclareLaunchArgument("Calibration_file",default_value=param_file_path))
    imu_transform_node=Node(
        package="perception",
        executable="imu_transform.py",
        name="ImuTransform_node",
        parameters=[{"imu_topic":LaunchConfiguration("imu_topic")},
                    {"imu_transformed_topic":LaunchConfiguration("imu_transformed_topic")},
                    {"imu_frame":LaunchConfiguration("imu_frame")},
                    {"lidar_frame":LaunchConfiguration("lidar_frame")},
                    {"pub_tf":True},
                    {"Calibration_file":LaunchConfiguration("Calibration_file")}
                    ],
        output="screen"
    )
    mid360_imu_normal=Node(
        package="perception",
        executable='imu_gravity_normal.py',
        name='mid360_imu_normal',
        parameters=[
            {"imu_topic":"/livox/imu"},
            {"output_topic":"/livox/imu/normal"}
        ],
        output="screen"
    )
    ld.add_action(imu_transform_node)
    ld.add_action(mid360_imu_normal)
    return ld