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
use_namespace="true"

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
        GroupAction([
            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=robot_name),

            DeclareLaunchArgument('robot_name', 
                                  default_value=robot_name,
            ),
            Node(
                package='gnss_poser',
                executable='gnss_poser',
                name='fix_imu2tfpose',
                output="screen",
                namespace="PoSLV",
                parameters=[{"coodinate_system":4}]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ekf_localization_launch_file_path),
                launch_arguments={'robot_name': robot_name}.items(),
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output="screen",
                parameters=[params]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ic120_navigation_launch_file_path),
                launch_arguments={'robot_name': robot_name}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ic120_track_pid_control_launch_file_path),
            ),
            #Node(
            #    package='ros2topic',
            #    executable='topic',
            #    name='cmd_spin_publisher',
            #    output="screen",
            #    arguments=['pub', '/ic120/cmd_spin', 'std_msgs/Bool', 'True', '-r', '1'],
            #),
        ]),
    ])
