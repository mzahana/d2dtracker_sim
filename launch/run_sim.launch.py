#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Start the Python node that runs the shell command
    node_cmd = Node(
        package='d2dtracker_sim',
        executable='gz_sim',
        output='screen'
    )

    # Run the ros_gz_bridge

    # Run some TF pkg

    ld.add_action(node_cmd)

    return ld