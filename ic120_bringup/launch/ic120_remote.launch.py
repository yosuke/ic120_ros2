import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

#plane=9
robot_name="ic120"
use_namespace=True

def generate_launch_description():
    
    ic120_description_dir=get_package_share_directory("ic120_description")
    ic120_navigation_dir=get_package_share_directory("ic120_navigation")
    gnss_localizer_ros2 = get_package_share_directory("gnss_localizer_ros2")
    ic120_unity_dir=get_package_share_directory("ic120_unity")
    xacro_model = os.path.join(ic120_description_dir, "urdf", "ic120.xacro")

    gnss_localizer_ros2_launch_file_path=os.path.join(gnss_localizer_ros2, "launch","gnss_localizer_ros2.py")
    ekf_localization_launch_file_path=os.path.join(ic120_navigation_dir,"launch","ekf_localization.launch.py")
    ic120_navigation_launch_file_path=os.path.join(ic120_navigation_dir,"launch","ic120_navigation.launch.py")
    ic120_standby_rviz_file = os.path.join(ic120_unity_dir, "rviz2", "ic120_standby.rviz")

    doc = xacro.parse(open(xacro_model))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    return LaunchDescription([

        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ekf_localization_launch_file_path),
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ic120_navigation_launch_file_path),
        ),

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(str(use_namespace)),
                namespace=robot_name),

            DeclareLaunchArgument('robot_name', default_value=robot_name,),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_map',
                arguments=['--x','21421.70', 
                           '--y','14020', 
                           '--z','68.62', 
                           '--roll','0', 
                           '--pitch','0', 
                           '--yaw','0', 
                           '--frame-id', 'world',
                           '--child-frame-id', 'map']),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[params]
            ),
            
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=["--display-config", ic120_standby_rviz_file]),
        ]),
    ])
