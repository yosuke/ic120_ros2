import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
import xacro

robot_name="ic120"

def generate_launch_description():
    ic120_description_dir=get_package_share_directory("ic120_description")
    ic120_navigation_dir=get_package_share_directory("ic120_navigation")
    ic120_track_pid_control_dir=get_package_share_directory("ic120_track_pid_control")
    xacro_model = os.path.join(ic120_description_dir, "urdf", "ic120.xacro")
    ekf_localization_launch_file_path=os.path.join(ic120_navigation_dir,"launch","ekf_localization.launch.py")
    ic120_navigation_launch_file_path=os.path.join(ic120_navigation_dir,"launch","ic120_navigation.launch.py")
    ic120_track_pid_control_launch_file_path=os.path.join(ic120_track_pid_control_dir,'launch',"ic120_track_pid_control.launch")
    doc = xacro.parse(open(xacro_model)) #xacroファイルをurfファイルに変換
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output="screen",
            arguments=['0.66', '0', '0', '0', '0', '0', robot_name + '_tf/base_link', robot_name + "_tf/base_link_rot"]
        ),
        Node(
            package='gnss_poser',
            name='fix_imu2tfpose',
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_localization_launch_file_path),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output="screen",
            parameters=[params]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ic120_navigation_launch_file_path),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ic120_track_pid_control_launch_file_path),
        ),
        #Node(
        #    package='ros2topic',
        #    executable='cmd_spin_publisher',
        #    output="screen",
        #    parameters=[params]
        #),
    ])
