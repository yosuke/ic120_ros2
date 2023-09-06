import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

robot_name="ic120"
use_namespace="true"

def generate_launch_description():
    ic120_description_dir=get_package_share_directory("ic120_description")
    xacro_model = os.path.join(ic120_description_dir, "urdf", "ic120.xacro")
    ic120_navigation_dir=get_package_share_directory('ic120_navigation')
    
    bringup_dir=get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, 'launch')

    ic120_ekf_yaml = LaunchConfiguration('ekf_yaml_file', default=os.path.join(get_package_share_directory('ic120_navigation'), 'config', 'ic120_ekf.yaml'))
    nav_params=LaunchConfiguration('nav_params', default=os.path.join(ic120_navigation_dir, 'params','navigation_parameters.yaml'))
    nav2_bringup_dir = os.path.join(get_package_share_directory('nav2_bringup'))
    nav2_bringup_launch_file_path = os.path.join(nav2_bringup_dir, 'launch' ,'navigation_launch.py')

    doc = xacro.parse(open(xacro_model)) #xacroファイルをurfファイルに変換
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    

    return LaunchDescription([
        GroupAction([
            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=robot_name),

            DeclareLaunchArgument('params_file', default_value=nav_params),
            DeclareLaunchArgument('use_sim_time', default_value='false'),

            Node(
                package='opera_tools',
                executable='odom_broadcaster',
                name='odom_broadcaster',
                output="screen",
                parameters=[{'odom_frame': "/ic120/odom"}]
            ),
            Node(
                package='opera_tools',
                executable='poseStamped2Odometry',
                name='poseStamped2ground_truth_odom',
                output="screen",
                parameters=[{'odom_header_frame': "/world",
                             'odom_child_frame': "/base_link",
                             'poseStamped_topic_name':"/ic120/base_link/pose",
                             'odom_topic_name':"/ic120/tracking/ground_truth"}]
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output="screen",
                parameters=[params]
            ),
            Node(
                package='robot_localization',
                executable='ekf_node',
                name="ekf_global",
                output="screen",
                remappings=[('odometry/filtered', '/ic120/odometry/global'),
                            ('odom0', '/ic120/odom'),
                            ('odom1', '/ic120/tracking/ground_truth')],
                parameters=[ic120_ekf_yaml,
                            {'map_frame' : "map",
                            'world_frame' : "map",
                            'odom_frame' : "odom",
                            'base_link_frame' : "base_link"}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup_launch_file_path),
                launch_arguments={'use_sim_time': use_sim_time,
                                  'param_files':nav_params}.items(),
            ),
        ]),
    ])
