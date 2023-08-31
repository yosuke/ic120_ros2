import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

robot_name="ic120"

def generate_launch_description():
    ic120_description_dir=get_package_share_directory("ic120_description")
    rviz_config=os.path.join(ic120_description_dir, "rviz2", "urdf.rviz")

    return LaunchDescription([
        GroupAction([
            PushRosNamespace(
                #condition=IfCondition(use_namespace),
                namespace=robot_name),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz_ic120",
                arguments=["--display-config", rviz_config],
                output="screen",
            ),
        ]),
    ])
