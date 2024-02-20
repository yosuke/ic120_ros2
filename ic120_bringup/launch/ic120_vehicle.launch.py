import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition
import xacro

robot_name="ic120"

def generate_launch_description():

    ic120_com3_ros_dir = get_package_share_directory("ic120_com3_ros")
    
    ic120_com3_ros_launch_file = os.path.join(ic120_com3_ros_dir, 'launch', 'ic120_com3_ros.launch.py')
    return LaunchDescription([

        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ic120_com3_ros_launch_file),
        ),
    ])
