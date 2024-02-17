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
                parameters=[{'poseStamped_topic_name':'/ic120/PoSLV/gnss_pose',
                             'odom_topic_name':'/ic120/PoSLV/gnss_odom',
                             'odom_child_frame':'gnss/base_link',
                             'odom_header_frame':'world'}],
            ),
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_global',
                output="screen",
                remappings=[('odometry/filtered','/ic120/odometry/global'),
                            ('odom0','/ic120/odom'),
                            ('odom1','/ic120/PoSLV/gnss_odom')], # GNSSのトピック名を確認すること
                parameters=[ic120_ekf_yaml_file,
                            {'map_frame' : 'map',
                            'odom_frame' : robot_name + '_tf/odom',
                            'base_link_frame' : robot_name + '_tf/base_link',
                            'world_frame' : 'map',
                            'use_sim_time' : False}]),
        ])
    ])

            # Node(
            #     package='robot_localization',
            #     executable='ekf_node',
            #     name="ekf_global",
            #     output="screen",
            #     remappings=[('odometry/filtered', '/ic120/odometry/global'),
            #                 ('odom0', '/ic120/odom'),
            #                 ('odom1', '/ic120/tracking/groundtruth')],
            #     parameters=[ic120_ekf_yaml_file,
            #                 {'map_frame' : "map",
            #                  'world_frame' : "map",
            #                   'odom_frame' : "ic120_tf/odom",
            #                   'base_link_frame' : "ic120_tf/base_link"}]),


# ROS1

#     <node pkg="robot_localization" type="ekf_localization_node" name="ekf_global" clear_params="true" output="screen">
#       <!-- <rosparam command="load" file="$(find ic120_unity)/config/ic120_ekf.yaml"/> -->
#       <rosparam command="load" file="$(find ic120_navigation)/config/ic120_ekf.yaml"/>
#       <param name="tf_prefix" value=""/>
#       <param name="map_frame" value="map"/>
#       <param name="world_frame" value="map"/>
#       <param name="odom_frame" value="$(arg robot_name)_tf/odom"/>
#       <param name="base_link_frame" value="$(arg robot_name)_tf/base_link"/>
#       <remap from="odometry/filtered" to="/$(arg robot_name)/odometry/global" />
#       <remap from="odom0" to="/$(arg robot_name)/odom" />
#       <remap from="odom1" to="/$(arg robot_name)/tracking/ground_truth" />
#     </node>


#   <node pkg="robot_localization" type="ekf_localization_node" name="ekf_global" clear_params="true" output="screen">
#     <rosparam command="load" file="$(find ic120_navigation)/config/ic120_ekf.yaml" />
#     <param name="tf_prefix" value=""/>
#     <param name="odom_frame" value="$(arg robot_name)_tf/odom"/>
#     <param name="base_link_frame" value="$(arg robot_name)_tf/base_link"/>
#     <remap from="odometry/filtered" to="odometry/global" />
#     <remap from="odom0" to="odom" />
#     <remap from="odom1" to="PoSLV/gnss_odom" />
#   </node>
# </launch>
