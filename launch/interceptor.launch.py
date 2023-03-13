#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Node for Drone 0
    model_name = {'gz_model_name': 'x500_d435'}
    autostart_id = {'px4_autostart_id': 4006}
    instance_id = {'instance_id': 1}
    xpos = {'xpos': 0.}
    ypos = {'ypos': 0.}
    zpos = {'zpos': 1.0}
    headless= {'headless' : 0}
    
    # Start the Python node that runs the shell command
    node_cmd = Node(
        package='d2dtracker_sim',
        executable='gz_sim',
        output='screen',
        name='px4_interceptor',
        parameters=[headless, model_name, autostart_id, instance_id, xpos, ypos, zpos]
    )

    # Run the ros_gz_bridge

    # Run some TF pkg to map from map -> local_pose_ENU

    ld.add_action(node_cmd)

    return ld