#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    robot_description_file = LaunchConfiguration('robot_description_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description_file',
        default_value=os.path.join(
            get_package_share_directory('my_package'),
            'config',
            'my_robot.urdf'
        ),
        description='Full path to robot description file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # Launch nodes
    robot_state_publisher = ExecuteProcess(
        cmd=[FindExecutable('robot_state_publisher'), '-p', robot_description_file],
        output='screen'
    )

    joint_state_publisher_gui = ExecuteProcess(
        cmd=[FindExecutable('joint_state_publisher_gui')],
        output='screen'
    )

    robot_controller = ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('my_package'), 'lib', 'my_package', 'robot_controller')],
        output='screen'
    )

    # Include launch files
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('robot_state_publisher'), 'launch', 'robot_state_publisher_launch.py')]),
        launch_arguments={'publish_frequency': '50',
                          'tf_prefix': 'my_robot'}.items())

    # Create launch description object
    ld = LaunchDescription()

    # Add nodes and arguments to launch description
    ld.add_action(declare_robot_description_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(robot_state_publisher_launch)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(robot_controller)

    return ld
