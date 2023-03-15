#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Node for Drone 0
    model_name = {'gz_model_name': 'x3_uav'}
    autostart_id = {'px4_autostart_id': 4007}
    instance_id = {'instance_id': 2}
    xpos = {'xpos': 0.0}
    ypos = {'ypos': 5.0}
    zpos = {'zpos': 0.5}
    headless= {'headless' : 0}

    # Namespace
    ns='target'
    
    # Start the Python node that runs the shell command
    node_cmd = Node(
        package='d2dtracker_sim',
        executable='gz_sim',
        namespace=ns,
        output='screen',
        name='px4_'+ns,
        parameters=[headless, model_name, autostart_id, instance_id, xpos, ypos, zpos]
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

    # Static TF map -> local_pose_ENU
    map2pose_tf_node = Node(
        package='tf2_ros',
        name='map2px4_'+ns+'_tf_node',
        executable='static_transform_publisher',
        arguments=[str(xpos['xpos']), str(ypos['ypos']), '0', '0', '0', '0', 'world', ns+'/'+enu_frame['parent_frame']],
    )

    ld.add_action(node_cmd)
    ld.add_action(tf_node)
    ld.add_action(map2pose_tf_node)

    return ld