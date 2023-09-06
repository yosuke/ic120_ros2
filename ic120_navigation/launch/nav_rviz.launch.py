import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition
import xacro

robot_name="ic120"
use_namespace="true"

def generate_launch_description():

    ic120_navigation_dir=get_package_share_directory("ic120_navigation")
    rviz_config = os.path.join(ic120_navigation_dir, "rviz", "navigation.rviz")
    xacro_model = os.path.join(get_package_share_directory("ic120_description"), "urdf", "ic120.xacro")

    doc = xacro.parse(open(xacro_model))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    return LaunchDescription([
        GroupAction([
            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=robot_name),

             DeclareLaunchArgument('use_gui', default_value="true"),
            DeclareLaunchArgument('config', default_value="robot"),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output="screen",
                namespace=robot_name,
                parameters=[params]
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz_ic120",
                arguments=["--display-config", rviz_config],
                output="screen",
                parameters=[{"tf_prefix":'ic120_tf'}],
            ),
        ]),
    ])
