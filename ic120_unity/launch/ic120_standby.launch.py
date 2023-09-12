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
    # ic120_unity_dir=get_package_share_directory("ic120_unity")
    # rviz_config = os.path.join(ic120_unity_dir, "rviz2", "ic120_standby.rviz")
    # ic120_unity_launch_file_path=os.path.join(ic120_unity_dir,"launch","ic120_unity.launch.py")
    nav2_bringup_dir=get_package_share_directory("nav2_bringup")
    ic120_unity_dir=get_package_share_directory("ic120_unity")
    ic120_navigation_dir=get_package_share_directory('ic120_navigation')

    rviz_config = os.path.join(ic120_unity_dir, "rviz2", "ic120_standby.rviz")
    nav2_bringup_launch_file_path=os.path.join(ic120_unity_dir,"launch","bringup_launch.py")
    ic120_ekf_yaml = LaunchConfiguration('ekf_yaml_file', default=os.path.join(get_package_share_directory('ic120_navigation'), 'config', 'ic120_ekf.yaml'))
    param_dir=os.path.join(ic120_navigation_dir, 'params','navigation_parameters_3.yaml')
    map_yaml_file=LaunchConfiguration('map', default=os.path.join(ic120_navigation_dir, 'map', 'map.yaml'))

    return LaunchDescription([

        
        DeclareLaunchArgument('/use_sim_time', default_value="true",),
        DeclareLaunchArgument('robot_name', default_value="ic120",),
        DeclareLaunchArgument('robot_name',default_value="ic120",),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='ic120',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ic120',
            namespace='ic120',
            arguments=['0', '3', '0', '0', '0', '0', 'map', 'ic120_tf/odom'],
        ),
        # Node(
        #         package='robot_localization',
        #         executable='ekf_node',
        #         name="ekf_global",
        #         output="screen",
        #         remappings=[('odometry/filtered', '/ic120/odometry/global'),
        #                     ('odom0', '/ic120/odom'),
        #                     ('odom1', '/ic120/tracking/ground_truth')],
        #         parameters=[ic120_ekf_yaml,
        #                     {'map_frame' : "map",
        #                      'world_frame' : "map",
        #                      'odom_frame' : "ic120_tf/odom",
        #                      'base_link_frame' : "ic120_tf/base_link"}],
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(ic120_unity_launch_file_path),
        #     launch_arguments={'init_pose': LaunchConfiguration('init_pose'),
        #                       'robot_name': LaunchConfiguration('robot_name')}.items()
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch_file_path),
            launch_arguments={
                        'map' : map_yaml_file,
                        'use_respawn' : 'true',
                        'use_sim_time': 'true',
                        'params_file': param_dir}.items(),
        ),


        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["--display-config", rviz_config],
        ),
    ])
