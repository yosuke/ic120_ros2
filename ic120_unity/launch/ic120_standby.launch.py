import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro

robot_name="ic120"

def generate_launch_description():
    ic120_unity_dir=get_package_share_directory("ic120_unity")
    rviz_config = os.path.join(ic120_unity_dir, "rviz2", "ic120_standby.rviz")
    ic120_unity_launch_file_path=os.path.join(ic120_unity_dir,"launch","ic120_unity.launch.py")

    return LaunchDescription([

        DeclareLaunchArgument('/use_sim_time', default_value="true",),
        DeclareLaunchArgument('robot_name', default_value="ic120",),
        DeclareLaunchArgument('init_pose', default_value="-x 0,-y 0,-z0",),
        DeclareLaunchArgument('robot_name',default_value="ic120",),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            namespace=robot_name,
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ic120_unity_launch_file_path),
            launch_arguments={'init_pose': LaunchConfiguration('init_pose'),
                              'robot_name': LaunchConfiguration('robot_name')}.items()
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["--display-config", rviz_config],
        ),
    ])
