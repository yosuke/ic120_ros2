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
use_sim_time="true"

def generate_launch_description():
    ic120_description_dir=get_package_share_directory("ic120_description")
    xacro_model = os.path.join(ic120_description_dir, "urdf", "ic120.xacro")
    ic120_navigation_dir=get_package_share_directory('ic120_navigation')
    
    bringup_dir=get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, 'launch')

    ic120_ekf_yaml = LaunchConfiguration('ekf_yaml_file', default=os.path.join(get_package_share_directory('ic120_navigation'), 'config', 'ic120_ekf.yaml'))
    behavior_server_params = LaunchConfiguration('behavior_server_params', default=os.path.join(ic120_navigation_dir, 'parameters','behavior_server_params.yaml'))
    bt_navigator_params = LaunchConfiguration('bt_navigator_params', default=os.path.join(ic120_navigation_dir, 'parameters','bt_navigator_params.yaml'))
    controller_server_params = os.path.join(ic120_navigation_dir, 'parameters','controller_server_params.yaml')
    global_costmap_params = LaunchConfiguration('global_costmap_params', default=os.path.join(ic120_navigation_dir, 'parameters','global_costmap_params.yaml'))
    local_costmap_params = LaunchConfiguration('local_costmap_params', default=os.path.join(ic120_navigation_dir, 'parameters','local_costmap_params.yaml'))
    map_server_params = LaunchConfiguration('map_server_params', default=os.path.join(ic120_navigation_dir, 'parameters','map_server_params.yaml'))
    planner_server_params = LaunchConfiguration('planner_server_params', default=os.path.join(ic120_navigation_dir, 'parameters','planner_server_params.yaml'))
    smoother_server_params = LaunchConfiguration('smoother_server_params', default=os.path.join(ic120_navigation_dir, 'parameters','smoother_server_params.yaml'))
    velocity_smoother_params = LaunchConfiguration('velocity_smoother_params', default=os.path.join(ic120_navigation_dir, 'parameters','velocity_smoother_params.yaml'))
    waypoint_follower_params = LaunchConfiguration('waypoint_follower_params', default=os.path.join(ic120_navigation_dir, 'parameters','waypoint_follower_params.yaml'))


    ic120_rviz_config = LaunchConfiguration('rviz_config_file', default=os.path.join(ic120_navigation_dir, 'rviz','nav2_default_view.rviz'))
    param_dir=LaunchConfiguration('params_file', default=os.path.join(ic120_navigation_dir, 'params','navigation_parameters.yaml'))
    nav2_bringup_dir = os.path.join(get_package_share_directory('nav2_bringup'))
    nav2_bringup_launch_file_path = os.path.join(nav2_bringup_dir, 'launch' ,'navigation_launch.py')

    doc = xacro.parse(open(xacro_model)) #xacroファイルをurfファイルに変換
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    
    

    return LaunchDescription([

        DeclareLaunchArgument('behavior_server_params', default_value=behavior_server_params),
        DeclareLaunchArgument('bt_navigator_params', default_value=bt_navigator_params),
        DeclareLaunchArgument('controller_server_params', default_value=controller_server_params),
        DeclareLaunchArgument('global_costmap_params', default_value=global_costmap_params),
        DeclareLaunchArgument('local_costmap_params', default_value=local_costmap_params),
        DeclareLaunchArgument('map_server_params', default_value=map_server_params),
        DeclareLaunchArgument('planner_server_params', default_value=planner_server_params),
        DeclareLaunchArgument('smoother_server_params', default_value=smoother_server_params),
        DeclareLaunchArgument('velocity_smoother_params', default_value=velocity_smoother_params),
        DeclareLaunchArgument('waypoint_follower_params', default_value=waypoint_follower_params),

        
        GroupAction([
            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=robot_name),

            Node(
                package='opera_tools',
                executable='odom_broadcaster',
                name='odom_broadcaster',
                output="screen",
                parameters=[{'odom_frame': "ic120/odom"}]
            ),
            # Node(
            #     package='opera_tools',
            #     executable='poseStamped2Odometry',
            #     name='poseStamped2ground_truth_odom',
            #     output="screen",
            #     parameters=[{'odom_header_frame': "world",
            #                  'odom_child_frame': "ic120_tf/base_link",
            #                  'poseStamped_topic_name':"/ic120/base_link/pose",
            #                  'odom_topic_name':"/ic120/tracking/ground_truth"}]
            # ),
            # Node(
            #     package='robot_state_publisher',
            #     executable='robot_state_publisher',
            #     output="screen",
            #     parameters=[params]
            # ),
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
            # IncludeLaunchDescription(
            #    PythonLaunchDescriptionSource(nav2_bringup_launch_file_path),
            #    launch_arguments={'use_sim_time': use_sim_time, 
            #                      'use_respawn' : 'true',
            #                      'params_file':param_dir}.items(),
            # ),
        #     Node(
        #         package='nav2_controller',
        #         executable='controller_server',
        #         name='controller_server',
        #         output='screen',
        #         parameters=[controller_server_params],
        #     ),
        #     Node(
        #         package='nav2_smoother',
        #         executable='smoother_server',
        #         name='smoother_server',
        #         output='screen',
        #         parameters=[smoother_server_params],
        #     ),
        #     Node(
        #         package='nav2_planner',
        #         executable='planner_server',
        #         name='planner_server',
        #         output='screen',
        #         parameters=[planner_server_params],
        #     ),
        #     Node(
        #         package='nav2_behaviors',
        #         executable='behavior_server',
        #         name='behavior_server',
        #         output='screen',
        #         parameters=[behavior_server_params],
        #     ),
        #     Node(
        #         package='nav2_bt_navigator',
        #         executable='bt_navigator',
        #         name='bt_navigator',
        #         output='screen',
        #         parameters=[bt_navigator_params],
        #     ),
        #     Node(
        #         package='nav2_waypoint_follower',
        #         executable='waypoint_follower',
        #         name='waypoint_follower',
        #         output='screen',
        #         parameters=[waypoint_follower_params],
        #     ),
        #     Node(
        #         package='nav2_velocity_smoother',
        #         executable='velocity_smoother',
        #         name='velocity_smoother',
        #         output='screen',
        #         parameters=[velocity_smoother_params],
        #     ),
        #     Node(
        #         package='nav2_lifecycle_manager',
        #         executable='lifecycle_manager',
        #         name='lifecycle_manager_navigation',
        #         output='screen',
        #         parameters=[{'use_sim_time': True,
        #                      'autostart': True,
        #                      'node_names': ['controller_server',
        #                                     'planner_server',
        #                                     'smoother_server',
        #                                     'bt_navigator',
        #                                     'waypoint_follower',
        #                                     'velocity_smoother',
        #                                     'behavior_server']}],
        #     ),
        ]),


    ])
