import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition
from launch_ros.actions import PushRosNamespace
import xacro

robot_name="ic120"
autostart='true'
use_respawn=True
use_namespace='true'

def generate_launch_description():
    # ic120_unity_dir=get_package_share_directory("ic120_unity")
    # rviz_config = os.path.join(ic120_unity_dir, "rviz2", "ic120_standby.rviz")
    # ic120_unity_launch_file_path=os.path.join(ic120_unity_dir,"launch","ic120_unity.launch.py")
    nav2_bringup_dir=get_package_share_directory("nav2_bringup")
    ic120_unity_dir=get_package_share_directory("ic120_unity")
    ic120_navigation_dir=get_package_share_directory('ic120_navigation')
    ic120_description_dir=get_package_share_directory("ic120_description")

    rviz_config = os.path.join(ic120_unity_dir, "rviz2", "ic120_standby.rviz")
    nav2_bringup_launch_file_path=os.path.join(ic120_unity_dir,"launch","bringup_launch.py")
    ic120_ekf_yaml = LaunchConfiguration('ekf_yaml_file', default=os.path.join(get_package_share_directory('ic120_navigation'), 'config', 'ic120_ekf.yaml'))
    param_dir=os.path.join(ic120_navigation_dir, 'params','navigation_parameters_3.yaml')
    map_yaml_file=LaunchConfiguration('map', default=os.path.join(ic120_navigation_dir, 'map', 'map.yaml'))
    params_file = os.path.join(ic120_navigation_dir, 'params', 'navigation_parameters_3.yaml')

    lifecycle_nodes_localization = [
                    'map_server',
                    'amcl']
    
    lifecycle_nodes_navigation = [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother']
    
    param_substitutions = {
        'use_sim_time': 'false',
        'yaml_filename': map_yaml_file}
    
    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key='ic120',
            param_rewrites=param_substitutions,
            convert_types=True)
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    xacro_model = os.path.join(ic120_description_dir, "urdf", "ic120.xacro")
    doc = xacro.parse(open(xacro_model)) #xacroファイルをurfファイルに変換
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    
    return LaunchDescription([

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=robot_name),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
            ),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='ic120',
                arguments=['0', '3', '0', '0', '0', '0', 'map', 'ic120_tf/odom'],
            ),
            Node(
                package='opera_tools',
                executable='odom_broadcaster',
                name='odom_broadcaster',
                output="screen",
            ),
            Node(
                package='opera_tools',
                executable='poseStamped2Odometry',
                name='poseStamped2ground_truth_odom',
                output="screen",
                parameters=[{'odom_header_frame': "world",
                                'odom_child_frame': "ic120_tf/base_link",
                                'poseStamped_topic_name':"/ic120/base_link/pose",
                                'odom_topic_name':"/ic120/tracking/ground_truth"}]
            ),
            # Node(
            #     package='robot_localization',
            #     executable='ekf_node',
            #     name="ekf_global",
            #     output="screen",
            #     remappings=[('odometry/filtered', '/ic120/odometry/global'),
            #                 ('odom0', '/ic120/odom'),
            #                 ('odom1', '/ic120/tracking/ground_truth')],
            #     parameters=[ic120_ekf_yaml,
            #                 {'map_frame' : "map",
            #                  'world_frame' : "map",
            #                  'odom_frame' : "ic120_tf/odom",
            #                  'base_link_frame' : "ic120_tf/base_link"}],
            # ),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output="screen",
                parameters=[params]
            ),
            Node(
                condition=IfCondition('true'),
                name='nav2_container',
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[configured_params, {'autostart': True}],
                remappings=remappings,
                output='screen'),
            
            # Localization packages
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),
            
            Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[{'use_sim_time': False},
                                {'autostart': True},
                                {'node_names': lifecycle_nodes_localization}]),

            # Navigation packages
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': False},
                            {'autostart': True},
                            {'node_names': lifecycle_nodes_navigation}]),
            

            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=["--display-config", rviz_config],
            ),
        ])
    ])
