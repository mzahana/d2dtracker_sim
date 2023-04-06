#!/usr/bin/env python3

"""
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
THIS SCRIPT DOES NOT WORK AS EXPECTED. DO NOT USE IT!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from launch.actions import (LogInfo, RegisterEventHandler)
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown

def generate_launch_description():
    ld = LaunchDescription()
    
    interceptor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('d2dtracker_sim'),
                'interceptor.launch.py'
            ])
        ])
    )

    target_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('d2dtracker_sim'),
                'target.launch.py'
            ])
        ])
    )

    # event = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=interceptor_launch,
    #         on_start=[
    #             LogInfo(msg='Spawning target...'),
    #             TimerAction(
    #                 period=2.0,
    #                 actions=[target_launch],
    #             )
    #         ]
    #     )
    # )


    ld.add_action(interceptor_launch)
    # This launches two Gazebo instances (GUIs)!
    # ld.add_action(target_launch)

    return ld