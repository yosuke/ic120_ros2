import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
import xacro

#plane=9
robot_name="ic120"
#use_namespace="true"

def generate_launch_description():
    ic120_handler_dir=get_package_share_directory("ic120_handler")
    g2_ros_dir=get_package_share_directory("g2_ros")
    ic120_joint_publisher_dir=get_package_share_directory("ic120_joint_publisher")
    ic120_handler_launch_file_path=os.path.join(ic120_handler_dir,'launch',"ic120_handler.launch")
    g2_ros_launch_file_path=os.path.join(g2_ros_dir,'launch',"g2_ros.launch")
    ic120_joint_publisher_launch_file_path=os.path.join(ic120_joint_publisher_dir,'launch',"ic120_joint_publisher.launch")
    return LaunchDescription([
        GroupAction([
            PushRosNamespace(
                #condition=IfCondition(use_namespace),
                namespace=robot_name
            ),
            DeclareLaunchArgument('robot_name', 
                                  default_value=robot_name,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ic120_handler_launch_file_path),
                launch_arguments={'robot_name': robot_name}.items(),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(g2_ros_launch_file_path),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ic120_joint_publisher_launch_file_path),
            ),
        ]),
    ])
