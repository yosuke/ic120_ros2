import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch_ros.actions import PushRosNamespace
import xacro

robot_name="ic120"
use_namespace="true"

def generate_launch_description():
    package_share_directory=get_package_share_directory("ic120_description")
    xacro_model = os.path.join(package_share_directory, "urdf", "ic120.xacro")
    
    bringup_dir=get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, 'launch')

    ic120_ekf_yaml = LaunchConfiguration('ekf_yaml_file', default=os.path.join(get_package_share_directory('ic120_navigation'), 'config', 'ic120_ekf.yaml'))
    map_nav_global_costmap_params_yaml = LaunchConfiguration('global_costmap_yaml_file', default=os.path.join(get_package_share_directory('ic120_navigation'), 'params','map_nav_params', 'global_costmap_params.yaml'))
    map_nav_local_costmap_params_yaml = LaunchConfiguration('local_costmap_yaml_file', default=os.path.join(get_package_share_directory('ic120_navigation'), 'params','map_nav_params', 'local_costmap_params.yaml'))
    global_costmap_params_yaml = LaunchConfiguration('global_costmap_yaml_file', default=os.path.join(get_package_share_directory('ic120_navigation'), 'params','odom_nav_params', 'global_costmap_params.yaml'))
    local_costmap_params_yaml = LaunchConfiguration('local_costmap_yaml_file', default=os.path.join(get_package_share_directory('ic120_navigation'), 'params','odom_nav_params', 'local_costmap_params.yaml'))
    base_local_planner_params_yaml = LaunchConfiguration('base_local_planner_yaml_file', default=os.path.join(get_package_share_directory('ic120_navigation'), 'params', 'base_local_planner_params.yaml'))
    base_global_planner_params_yaml = LaunchConfiguration('base_global_planner_yaml_file', default=os.path.join(get_package_share_directory('ic120_navigation'), 'params', 'base_global_planner_params.yaml'))
    costmap_common_params_yaml = LaunchConfiguration('costmap_common_yaml_file', default=os.path.join(get_package_share_directory('ic120_navigation'), 'costmap_common_params.yaml'))
    move_base_params_yaml = LaunchConfiguration('move_base_yaml_file', default=os.path.join(get_package_share_directory('ic120_navigation'), 'params','move_base_params.yaml'))
    
    doc = xacro.parse(open(xacro_model)) #xacroファイルをurfファイルに変換
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    return LaunchDescription([
        GroupAction([
            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=robot_name),
                DeclareLaunchArgument(
                    'ic120_ekf_yaml',
                    default_value=ic120_ekf_yaml,
                ),

            DeclareLaunchArgument(
                'map_nav_global_costmap_params_yaml',
                default_value=map_nav_global_costmap_params_yaml,
            ),

            DeclareLaunchArgument(
                'map_nav_local_costmap_params_yaml',
                default_value=map_nav_local_costmap_params_yaml,
            ),

            DeclareLaunchArgument(
                'global_costmap_params_yaml',
                default_value=global_costmap_params_yaml,
            ),

            DeclareLaunchArgument(
                'odom_nav_local_costmap_params_yaml',
                default_value=local_costmap_params_yaml,
            ),

            DeclareLaunchArgument(
                'base_local_planner_params_yaml',
                default_value=base_local_planner_params_yaml,
            ),
            
            DeclareLaunchArgument(
                'base_global_planner_params_yaml',
                default_value=base_global_planner_params_yaml,
            ), 

            DeclareLaunchArgument(
                'costmap_common_params_yaml',
                default_value=costmap_common_params_yaml,
            ),
            
            DeclareLaunchArgument(
                'move_base_params_yaml',
                default_value=move_base_params_yaml,
            ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                output="screen",
                parameters=[params]
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output="screen",
                parameters=[params]
            ),
            Node(
                package='opera_tools',
                executable='odom_broadcaster',
                name='odom_broadcaster',
                output="screen",
                parameters=[{'odom_frame': robot_name+"_tf/odom", 'base_link_frame':robot_name+"_tf/base_link"}]
            ),
            Node(
                package='opera_tools',
                executable='poseStamped2Odometry',
                name='poseStamped2ground_truth_odom',
                output="screen",
                parameters=[{'odom_header_frame': "world", 'odom_child_frame': robot_name+"_tf/base_link",'poseStamped_topic_name':"base_link/pose", 'odom_topic_name':"tracking/ground_truth"}]
            ),
            Node(
                package='robot_localization',
                executable='ekf_node',
                name="ekf_global",
                output="screen",
                parameters=[ic120_ekf_yaml,
                            {'tf_prefix' : "",
                            'map_frame' : "map",
                            'world_frame' : "map",
                            'odom_frame' : robot_name + "_tf/odom",
                            'base_link_frame' : robot_name + "_tf/base_link",
                            'remappings':{'odometry/filtered':'/'+robot_name+'/odometry/global',
                                        'odom0':'/'+robot_name+'/odom',
                                        'odom1':'/'+robot_name+'/tracking/ground_truth'}}
                ]
            ),
            # local costmap
            Node(
                package='nav2_costmap_2d',
                executable='nav2_costmap_2d',
                name = 'local_costmap',
                output="screen",
                parameters = [local_costmap_params_yaml]
            ),
            # global costmap
            Node(
                package='nav2_map_server',
                executable='map_server',
                name = 'global_costmap',
                output="screen",
                parameters = [global_costmap_params_yaml]
            ),
            # local planner
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='base_local_planner',
                output="screen",
                parameters=[base_local_planner_params_yaml],
            ),
            #global planner
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='base_global_planner',
                output="screen",
                parameters=[base_global_planner_params_yaml],
            ),
            Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='nav2_lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['local_costmap','global_costmap','base_local_planner', 'base_global_planner']}],
            ),
        ]),
    ])
