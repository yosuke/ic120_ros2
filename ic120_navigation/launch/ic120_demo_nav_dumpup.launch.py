import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

robot_name="ic120"

def generate_launch_description():
    ic120_navigation_dir=get_package_share_directory("ic120_navigation")
    ic120_ekf_yaml = LaunchConfiguration('ekf_yaml_file', ic120_navigation_dir, 'config', 'ic120_ekf.yaml')
    xacro_model = os.path.join(get_package_share_directory("ic120_description"), "urdf", "ic120.xacro")
    rviz_config = os.path.join(ic120_navigation_dir, "rviz", "zx120_ic120_demo.rviz")

    doc = xacro.parse(open(xacro_model))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    return LaunchDescription([
        
        Node(
            package='ic120_navigation',
            executable='dump_navigation',
            name='dunmp_navigation'  
        ),
        Node(
            package='ic120_navigation',
            executable='dump_srv_server',
            name='dump_srv_server'  
        ),
        Node(
            package='robot_state_publisher',
           executable='robot_state_publisher',
            output="screen",
            parameters=[params]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["--display-config", rviz_config],
            output="screen",
        ),
    ])