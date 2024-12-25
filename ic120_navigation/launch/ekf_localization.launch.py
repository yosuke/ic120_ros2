import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition

robot_name="ic120"
use_namespace=True

def generate_launch_description():

    ic120_description_dir=get_package_share_directory("ic120_description")
    ic120_navigation_dir=get_package_share_directory("ic120_navigation")

    ic120_ekf_yaml_file = LaunchConfiguration('ekf_yaml_file', default=os.path.join(ic120_navigation_dir, 'config', 'ic120_ekf.yaml'))

    return LaunchDescription([

        DeclareLaunchArgument('robot_name', default_value='ic120'),

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(str(use_namespace)),
                namespace=robot_name),
            Node(
                package="ic120_navigation",
                executable="poseStamped2Odometry",
                name="poseStamped2Odometry",
                parameters=[{'poseStamped_topic_name':'/ic120/global_pose',
                             'odom_topic_name':'/ic120/gnss_odom',
                             'odom_child_frame':'gnss',
                             'odom_header_frame':'world'}],
            ),
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_global',
                output="screen",
                remappings=[('odometry/filtered','/ic120/odometry/global'),
                            ('odom0','/ic120/odom_pose'),
                            ('odom1','/ic120/gnss_odom')], # GNSSのトピック名を確認すること
                parameters=[ic120_ekf_yaml_file,
                                            {
                                            'debug': False,
                                            'frequency': 10.0,
                                            'transform_time_offset': 0.0,
                                            'transform_timeout': 0.0,
                                            'print_diagnostics': True,
                                            'publish_acceleration': True,
                                            'print_diagnostics': True,
                                            'publish_tf': True,
                                            'two_d_mode': True ,                                               
                                                'map_frame' : 'map',
                                            'odom_frame' : robot_name + '_tf/odom',
                                            'base_link_frame' : robot_name + '_tf/base_link',
                                            'world_frame' : 'map',
                                            'use_sim_time' : False,
                                            'odom0' : '/ic120/odom_pose',
                                            'odom0_config': [
                                                True,  True,  False,
                                                False, False, True,
                                                False, False, False,
                                                False, False, False,
                                                False, False, False],
                                            'odom0_differential': False,
                                            'odom1' : '/ic120/gnss_odom',
                                            'odom1_config': [
                                                True,  True,  True,
                                                False, False, True,
                                                False, False, False,
                                                False, False, False,
                                                False, False, False],
                                            'odom1_differential': False,
                                            }]
            ),
        ])
    ])
