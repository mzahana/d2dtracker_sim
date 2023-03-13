#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Node for Drone 0
    d_1_model_name = {'gz_model_name': 'x500_d435'}
    d_1_autostart_id = {'px4_autostart_id': 4006}
    d_1_instance_id = {'instance_id': 0}
    d_1_xpos = {'xpos': 0.}
    d_1_ypos = {'ypos': 0.}
    d_1_zpos = {'zpos': 1.0}
    headless= {'headless' : 0}
    
    # Start the Python node that runs the shell command
    node1_cmd = Node(
        package='d2dtracker_sim',
        executable='gz_sim',
        output='screen',
        name='px4_uav0',
        parameters=[headless, d_1_model_name, d_1_autostart_id, d_1_instance_id, d_1_xpos, d_1_ypos, d_1_zpos]
    )

    # Node for Drone 0
    # d_1_model_name = {'gz_model_name': 'x500_d435'}
    # d_1_autostart_id = {'px4_autostart_id': 4006}
    # d_1_instance_id = {'instance_id': 1}
    # d_1_xpos = {'xpos': 5.}
    # d_1_ypos = {'ypos': 0.}
    # d_1_zpos = {'zpos': 1.0}
    # headless= {'headless' : 1}
    
    # # Start the Python node that runs the shell command
    # node2_cmd = Node(
    #     package='d2dtracker_sim',
    #     executable='gz_sim',
    #     output='screen',
    #     name='px4_uav1',
    #     parameters=[headless, d_1_model_name, d_1_autostart_id, d_1_instance_id, d_1_xpos, d_1_ypos, d_1_zpos]
    # )

    # Run the ros_gz_bridge

    # Run some TF pkg

    ld.add_action(node1_cmd)
    # ld.add_action(node2_cmd)

    return ld