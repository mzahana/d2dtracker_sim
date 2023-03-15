#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Node for Drone 0
    model_name = {'gz_model_name': 'x500_d435'}
    autostart_id = {'px4_autostart_id': 4006}
    instance_id = {'instance_id': 1}
    xpos = {'xpos': 2.}
    ypos = {'ypos': 0.}
    zpos = {'zpos': 0.5}
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

    # TF odom NED -> ENU
    enu_frame= {'parent_frame' : 'local_pose_ENU'}
    base_link= {'child_frame' : 'base_link'}
    tf_node = Node(
        package='d2dtracker_sim',
        executable='tf_node',
        output='screen',
        name='interceptor_tf_node',
        namespace='px4_'+str(instance_id['instance_id']),
        parameters=[enu_frame, base_link]
    )

    # Static TF map -> local_pose_ENU
    map2pose_tf_node = Node(
        package='tf2_ros',
        name='map2px4_'+str(instance_id['instance_id'])+'_tf_node',
        executable='static_transform_publisher',
        arguments=[str(xpos['xpos']), str(ypos['ypos']), '0', '0', '0', '0', 'world', 'px4_'+str(instance_id['instance_id'])+'/'+enu_frame['parent_frame']],
    )

    # Static TF base_link -> depth_camera
    # .15 0 .25 0 0 1.5707
    cam_x = 0.15
    cam_y = 0.0
    cam_z = 0.25
    cam_roll = 0.0
    cam_pitch = 0.0
    cam_yaw = 0.0
    cam_tf_node = Node(
        package='tf2_ros',
        name='depth_'+str(instance_id['instance_id'])+'_tf_node',
        executable='static_transform_publisher',
        arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), 'px4_'+str(instance_id['instance_id'])+'/'+base_link['child_frame'], 'px4_'+str(instance_id['instance_id'])+'/depth_camera'],
    )

    # Transport rgb and depth images from GZ topics to ROS topics    
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        name='ros_bridge_node_depthcam',
        executable='parameter_bridge',
        arguments=['/d435/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '/d435/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '--ros-args', '-r', '/d435/depth_image:='+'/px4_'+str(instance_id['instance_id'])+'/depth_image',
                   '-r', '/d435/image:='+'/px4_'+str(instance_id['instance_id'])+'/image'
                   ],
    )

    # Midro DDS agent
    port={'port':8888}
    dds_agent_node = Node(
        package='d2dtracker_sim',
        executable='microdds',
        output='screen',
        name='microdds_node',
        parameters=[port]
    )

    # Rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='sim_rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('d2dtracker_sim'), 'sim.rviz')]
    )

    ld.add_action(node_cmd)
    ld.add_action(tf_node)
    ld.add_action(map2pose_tf_node)
    ld.add_action(cam_tf_node)
    ld.add_action(ros_gz_bridge)
    ld.add_action(dds_agent_node)
    ld.add_action(rviz_node)

    return ld