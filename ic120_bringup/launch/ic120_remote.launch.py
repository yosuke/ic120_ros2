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
    ic120_unity_dir=get_package_share_directory("ic120_unity")
    ic120_track_pid_control_dir=get_package_share_directory("ic120_track_pid_control")
    xacro_model = os.path.join(ic120_description_dir, "urdf", "ic120.xacro")

    ekf_localization_launch_file_path=os.path.join(ic120_navigation_dir,"launch","ekf_localization.launch.py")
    ic120_navigation_launch_file_path=os.path.join(ic120_navigation_dir,"launch","ic120_navigation.launch.py")
    ic120_track_pid_control_launch_file_path=os.path.join(ic120_track_pid_control_dir,'launch',"ic120_track_pid_control.launch.py")
    ic120_standby_rviz_file = os.path.join(ic120_unity_dir, "rviz2", "ic120_standby.rviz")

    doc = xacro.parse(open(xacro_model))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    return LaunchDescription([
        Node(
            package='ic120_unity',
            executable='convert_goal_pose',
            name='convert_goal_pose',
            output="screen"),
        Node(
            package='ic120_unity',
            executable='convert_initial_pose',
            name='convert_initial_pose',
            output="screen"),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_offset_tf',
            output="screen",
            namespace=robot_name,
            arguments=['--x','0.66', 
                       '--y','0', 
                       '--z','0', 
                       '--yqw','0', 
                       '--pitch','0', 
                       '--roll','0', 
                       '--frame-id', robot_name + '_tf/base_link',
                       '--child-frame-id', robot_name + "_tf/base_link_rot"]),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ekf_localization_launch_file_path),
                launch_arguments={'robot_name': robot_name}.items(),
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ic120_navigation_launch_file_path),
                launch_arguments={'robot_name': robot_name}.items(),
        ),

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(str(use_namespace)),
                namespace=robot_name),

            DeclareLaunchArgument('robot_name', default_value=robot_name,),
            #DeclareLaunchArgument('plane', default_value=9,),

            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(ic120_track_pid_control_launch_file_path),
            # ),

            # Node(
            #     package='gnss_poser',
            #     executable='gnss_poser',
            #     name='fix_imu2tfpose',
            #     output="log",
            #     namespace="PoSLV",
            #     parameters=[{"coodinate_system":4}]
            # ),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[params]
            ),
            
            Node(
                package='ic120_bringup',
                executable='rostopic',
                name='cmd_spin_publisher',
                output="screen",
                parameters=[{'topic_name':'/ic120/cmd_spin',
                             'time':1,
                             'value':True}]
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=["--display-config", ic120_standby_rviz_file]),
        ]),
    ])
