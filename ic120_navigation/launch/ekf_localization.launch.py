import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

robot_name="ic120"

def generate_launch_description():
    ic120_description_dir=get_package_share_directory("ic120_description")
    ic120_navigation_dir=get_package_share_directory("ic120_navigation")
    ic120_ekf_yaml = LaunchConfiguration('ekf_yaml_file', default=os.path.join(get_package_share_directory('ic120_navigation'), 'config', 'ic120_ekf.yaml'))

    return LaunchDescription([

        DeclareLaunchArgument(
            'ic120_ekf_yaml',
            default_value=ic120_ekf_yaml,
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output="screen",
            parameters=[ic120_ekf_yaml,
                        {'tf_prefix' : "",
                         'odom_frame' : robot_name + "_tf/odom",
                         'base_link_frame' : robot_name + "_tf/base_link",
                         'remappings':{'odometry/filtered':'odometry/global',
                                       'odom0':'odom',
                                       'odom1':'PoSLV/gnss_odom'}}
            ]
        ),
        Node(
            package='ic120_navigation',
            executable='dump_navigation',
            name='dunmp_navigation'  
        ),
    ])