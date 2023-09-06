import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction, DeclareLaunchArgument

robot_name="ic120"
use_rviz="true"
use_namespace="true"

def generate_launch_description():
    ic120_description_dir=get_package_share_directory("ic120_description")
    rviz_config=os.path.join(ic120_description_dir, "rviz2", "urdf.rviz")

    return LaunchDescription([
        GroupAction([
            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=robot_name),

            DeclareLaunchArgument('use_rviz',default_value='true'),

            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz_ic120",
                condition=IfCondition(LaunchConfiguration('use_rviz')),
                arguments=["--display-config", rviz_config],
            ),
        ]),
    ])
