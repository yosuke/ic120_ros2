import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    ic120_unity_dir=get_package_share_directory("ic120_unity")
    rviz_config = os.path.join(ic120_unity_dir, "rviz2", "ic120_standby.rviz")
    ic120_unity_launch_file_path=os.path.join(ic120_unity_dir,"launch","ic120_unity.launch.py")

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ic120_unity_launch_file_path),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["--display-config", rviz_config],
            output="screen",
        ),
    ])
