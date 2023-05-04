#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ld = LaunchDescription()

    # Node for Drone 2
    model_name = {'gz_model_name': 'x3_uav'}
    autostart_id = {'px4_autostart_id': '4021'}
    instance_id = {'instance_id': '2'}
    xpos = {'xpos': '4.0'}
    ypos = {'ypos': '0.0'}
    zpos = {'zpos': '0.5'}
    headless= {'headless' : '0'}

    # Namespace
    ns='target'

    # PX4 SITL + Spawn x3
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('d2dtracker_sim'),
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_ns': ns,
            'headless': headless['headless'],
            'gz_model_name': model_name['gz_model_name'],
            'px4_autostart_id': autostart_id['px4_autostart_id'],
            'instance_id': instance_id['instance_id'],
            'xpos': xpos['xpos'],
            'ypos': ypos['ypos'],
            'zpos': zpos['zpos']
        }.items()
    )

    # TF odom NED -> ENU
    enu_frame= {'parent_frame' : 'local_pose_ENU'}
    base_link= {'child_frame' : 'base_link'}
    tf_node = Node(
        package='d2dtracker_sim',
        executable='tf_node',
        output='screen',
        name=ns+'_ned2enu_tf',
        namespace=ns,
        parameters=[enu_frame, base_link]
    )

    enu_frame= {'odom_frame' : 'local_pose_ENU'}
    base_link= {'baselink_frame' : 'base_link'}
    tf_period= {'tf_pub_period' : 0.02}
    px4_ros_node = Node(
        package='px4_ros_com',
        executable='px4_ros',
        output='screen',
        name=ns+'_px4_ros_com',
        namespace=ns,
        parameters=[enu_frame, base_link, tf_period]
    )

    # Static TF map(or world) -> local_pose_ENU
    map2pose_tf_node = Node(
        package='tf2_ros',
        name='map2px4_'+ns+'_tf_node',
        executable='static_transform_publisher',
        arguments=[str(xpos['xpos']), str(ypos['ypos']), '0', '0', '0', '0', 'world', ns+'/'+enu_frame['odom_frame']],
    )

    ld.add_action(gz_launch)
    ld.add_action(px4_ros_node)
    ld.add_action(map2pose_tf_node)

    return ld