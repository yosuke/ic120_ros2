import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory(
            "ic120_description"), "rviz2", "urdf.rviz")

    xacro_model = os.path.join(get_package_share_directory("ic120_description"), "urdf", "ic120.xacro")

    doc = xacro.parse(open(xacro_model))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    return LaunchDescription([
        DeclareLaunchArgument('gui', 
                               default_value="true",
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output="screen",
            condition=IfCondition(LaunchConfiguration('gui')),
            parameters=[params]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output="screen",
            condition=UnlessCondition(LaunchConfiguration('gui')),
            parameters=[params]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output="screen",
            parameters=[params]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["--display-config", rviz_config],
            output="screen",
        ),
    ])
