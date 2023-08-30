import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

robot_name="ic120"

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

    ])
