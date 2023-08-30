import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import xacro

robot_name="ic120"

def generate_launch_description():
    ic120_description_dir=get_package_share_directory("ic120_description")
    rviz_config=os.path.join(ic120_description_dir, "rviz2", "urdf.rviz")

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["--display-config", rviz_config],
            output="screen",
        ),
        
    ])
