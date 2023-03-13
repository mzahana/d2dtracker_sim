#!/usr/bin/env python3

"""
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
THIS SCRIPT DOES NOT WORK AS EXPECTED. DO NOT USE IT!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Get the path to the package containing the launch files
    my_package_dir = get_package_share_directory('d2dtracker_sim')
    
    # Define the paths to the other launch files
    target_launch_file = os.path.join(my_package_dir, 'target.launch.py')
    interceptor_launch_file = os.path.join(my_package_dir, 'interceptor.launch.py')


    interceptor_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(interceptor_launch_file))
    ld.add_action(interceptor_launch)

    # target_launch = IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource(target_launch_file))
    # ld.add_action(target_launch)

    return ld