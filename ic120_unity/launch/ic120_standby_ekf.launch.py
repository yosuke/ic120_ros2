import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition


robot_name="ic120"
use_autostart=True
use_sim_time=True
use_respawn=True
use_namespace=True

def generate_launch_description():

    ic120_unity_dir=get_package_share_directory("ic120_unity")
    ic120_navigation_dir=get_package_share_directory('ic120_navigation')
    ic120_description_dir=get_package_share_directory("ic120_description")

    ic120_standby_rviz_file = os.path.join(ic120_unity_dir, "rviz2", "ic120_standby.rviz")
    navigation_parameters_yaml_file = os.path.join(ic120_navigation_dir, 'params', 'navigation_parameters.yaml')
    ic120_xacro_file = os.path.join(ic120_description_dir, "urdf", "ic120.xacro")

    ic120_ekf_yaml_file = LaunchConfiguration('ekf_yaml_file', default=os.path.join(ic120_navigation_dir, 'config', 'ic120_ekf.yaml'))
    map_yaml_file=LaunchConfiguration('map', default=os.path.join(ic120_navigation_dir, 'map', 'map.yaml'))

    lifecycle_nodes_localization = [
                   'map_server']
    
    lifecycle_nodes_navigation = [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother']
    
    param_substitutions = {
        'use_sim_time': str(use_sim_time),
        'yaml_filename': map_yaml_file}
    
    configured_params = RewrittenYaml(
            source_file=navigation_parameters_yaml_file,
            root_key='ic120',
            param_rewrites=param_substitutions,
            convert_types=True)
    
    #remappings = [('/tf', 'tf'),
    #              ('/tf_static', 'tf_static')]
    
    remappings_ic120_tf=[('/ic120/tf','tf'),
                         ('/ic120/tf_static', 'tf_static')]
    
    
    doc = xacro.parse(open(ic120_xacro_file)) 
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    
    return LaunchDescription([

        Node(
            package='ic120_unity',
            executable='convert_goal_pose',
            name='convert_goal_pose',
            output="screen"),

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(str(use_namespace)),
                namespace=robot_name),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_map',
                arguments=['--x','0', 
                           '--y','0', 
                           '--z','0', 
                           '--roll','0', 
                           '--pitch','0', 
                           '--yaw','0', 
                           '--frame-id', 'world',
                           '--child-frame-id', 'map']),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom',
                arguments=['--x','0', 
                           '--y','3', 
                           '--z','0', 
                           '--roll','0', 
                           '--pitch','0', 
                           '--yaw','0', 
                           '--frame-id', 'map',
                           '--child-frame-id', 'ic120_tf/odom']), 
            Node(
                package='opera_tools',
                executable='odom_broadcaster',
                name='odom_broadcaster',
                output="screen"),
            Node(
                package='opera_tools',
                executable='poseStamped2Odometry',
                name='poseStamped2ground_truth_odom',
                output="screen",
                parameters=[{'odom_header_frame': "world",
                                'odom_child_frame': "ic120_tf/base_link",
                                'poseStamped_topic_name':"/ic120/base_link/pose",
                                'odom_topic_name':"/ic120/tracking/ground_truth"}]),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output="screen",
                parameters=[params],
                remappings=remappings_ic120_tf),   
            Node(
                condition=IfCondition('true'),
                name='nav2_container',
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[configured_params, {'autostart': use_autostart}],
                remappings=remappings_ic120_tf,
                output='screen'),
            
            #########################
            # Localization packages #
            #########################

            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings_ic120_tf),
            Node(
                package='robot_localization',
                executable='ekf_node',
                name="ekf_global",
                output="screen",
                remappings=[('odometry/filtered', '/ic120/odometry/global'),
                            ('odom0', '/ic120/odom'),
                            ('odom1', '/ic120/tracking/ground_truth')],
                parameters=[ic120_ekf_yaml_file,
                            {'map_frame' : "map",
                             'world_frame' : "map",
                             'odom_frame' : "ic120_tf/odom",
                              'base_link_frame' : "ic120_tf/base_link"}]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': use_autostart},
                            {'node_names': lifecycle_nodes_localization}]),

            #######################
            # Navigation packages #
            #######################

            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings_ic120_tf + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings_ic120_tf),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings_ic120_tf),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings_ic120_tf +
                           [('cmd_vel', 'tracks/cmd_vel')]),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings_ic120_tf+[('ic120/goal_pose','goal_pose')]),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings_ic120_tf),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings_ic120_tf +
                        [('cmd_vel', 'cmd_vel_nav'), 
                         ('cmd_vel_smoothed', 'tracks/cmd_vel')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': use_autostart},
                            {'node_names': lifecycle_nodes_navigation}]),

            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=["--display-config", ic120_standby_rviz_file]),
            
        ])
    ])
