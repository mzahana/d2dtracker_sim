#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from math import radians
def generate_launch_description():
    ld = LaunchDescription()

    ns='interceptor'

    # Node for Drone 1
    world = {'gz_world': 'default'}
    model_name = {'gz_model_name': 'x500_d435'}
    autostart_id = {'px4_autostart_id': '4020'}
    instance_id = {'instance_id': '1'}
    xpos = {'xpos': '0.0'}
    ypos = {'ypos': '0.0'}
    zpos = {'zpos': '0.5'}
    headless= {'headless' : '0'}

    # PX4 SITL + Spawn x500_d435
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
            'gz_world': world['gz_world'],
            'gz_model_name': model_name['gz_model_name'],
            'px4_autostart_id': autostart_id['px4_autostart_id'],
            'instance_id': instance_id['instance_id'],
            'xpos': xpos['xpos'],
            'ypos': ypos['ypos'],
            'zpos': zpos['zpos']
        }.items()
    )

    # MicroXRCEAgent
    xrce_agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('d2dtracker_sim'),
                'xrce_agent.launch.py'
            ])
        ]),
        launch_arguments={
            'port': '8888'
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

    # Static TF map -> local_pose_ENU
    map2pose_tf_node = Node(
        package='tf2_ros',
        name='map2px4_'+ns+'_tf_node',
        executable='static_transform_publisher',
        arguments=[str(xpos['xpos']), str(ypos['ypos']), '0', '0.0', '0', '0', 'world', ns+'/'+enu_frame['odom_frame']],
    )

    # Static TF base_link -> depth_camera
    # .15 0 .25 0 0 1.5707
    cam_x = 0.15
    cam_y = 0.0
    cam_z = 0.25
    cam_roll = radians(-90.0)
    cam_pitch = 0.0
    cam_yaw = radians(-90.0)
    cam_tf_node = Node(
        package='tf2_ros',
        name=ns+'_base2depth_tf_node',
        executable='static_transform_publisher',
        # arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), ns+'/'+base_link['child_frame'], ns+'/depth_camera'],
        arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), ns+'/'+base_link['baselink_frame'], 'x500_d435_1/link/realsense_d435'],
        
    )

    # Transport rgb and depth images from GZ topics to ROS topics    
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        name='ros_bridge_node_depthcam',
        executable='parameter_bridge',
        arguments=['/d435/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '/d435/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '/d435/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                   '/d435/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                   '--ros-args', '-r', '/d435/depth_image:='+ns+'/depth_image',
                   '-r', '/d435/image:='+ns+'/image',
                   '-r', '/d435/points:='+ns+'/points',
                   '-r', '/d435/camera_info:='+ns+'/camera_info'
                   ],
    )

    # Drone detector
    drone_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('d2dtracker_drone_detector'),
                'detection.launch.py'
            ])
        ]),
        launch_arguments={
            'detections_topic': 'detections_poses',
            'depth_topic' : 'interceptor/depth_image',
            'detector_ns': ''
        }.items()
    )

    # Kalman filter
    kf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('multi_target_kf'),
                'launch/kf_const_vel.launch.py'
            ])
        ]),
        launch_arguments={
            'detections_topic': 'detections_poses',
            'kf_ns' : ''
        }.items()
    )
    
    # Rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='sim_rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('d2dtracker_sim'), 'sim.rviz')]
    )

    ld.add_action(gz_launch)
    # ld.add_action(tf_node)
    ld.add_action(map2pose_tf_node)
    ld.add_action(cam_tf_node)
    ld.add_action(ros_gz_bridge)
    ld.add_action(rviz_node)
    ld.add_action(xrce_agent_launch)
    ld.add_action(drone_detection_launch)
    ld.add_action(kf_launch)
    ld.add_action(px4_ros_node)

    return ld