import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
import xacro

plane=9
robot_name="ic120"

def generate_launch_description():
    ic120_handler_dir=get_package_share_directory("ic120_handler")
    g2_ros_dir=get_package_share_directory("g2_ros")
    ic120_joint_publisher_dir=get_package_share_directory("ic120_joint_publisher")
    ic120_handler_launch_file_path=os.path.join(ic120_handler_dir,'launch',"ic120_handler.launch")
    g2_ros_launch_file_path=os.path.join(g2_ros_dir,'launch',"g2_ros.launch")
    ic120_joint_publisher_launch_file_path=os.path.join(ic120_joint_publisher_dir,'launch',"ic120_joint_publisher.launch")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ic120_handler_launch_file_path),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(g2_ros_launch_file_path),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ic120_joint_publisher_launch_file_path),
        ),
    ])
