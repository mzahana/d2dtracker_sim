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
    zpos = {'zpos': '0.1'}
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
            'zpos': '0.0'
        }.items()
    )

    # MicroXRCEAgent
    # xrce_agent_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('d2dtracker_sim'),
    #             'xrce_agent.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'port': '8888'
    #     }.items()
    # )

    # MAVROS
    file_name = 'interceptor_px4_pluginlists.yaml'
    package_share_directory = get_package_share_directory('d2dtracker_sim')
    plugins_file_path = os.path.join(package_share_directory, file_name)
    file_name = 'interceptor_px4_config.yaml'
    config_file_path = os.path.join(package_share_directory, file_name)
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('d2dtracker_sim'),
                'mavros.launch.py'
            ])
        ]),
        launch_arguments={
            'mavros_namespace' :ns+'/mavros',
            'tgt_system': '2',
            'fcu_url': 'udp://:14541@127.0.0.1:14558',
            'pluginlists_yaml': plugins_file_path,
            'config_yaml': config_file_path,
            'base_link_frame': 'interceptor/base_link',
            'odom_frame': 'interceptor/odom',
            'map_frame': 'map'
        }.items()
    )

    odom_frame = 'odom'
    base_link_frame=  'base_link'

    # Static TF map/world -> local_pose_ENU
    map_frame='map'
    map2pose_tf_node = Node(
        package='tf2_ros',
        name='map2px4_'+ns+'_tf_node',
        executable='static_transform_publisher',
        arguments=[str(xpos['xpos']), str(ypos['ypos']), '0', '0.0', '0', '0', map_frame, ns+'/'+odom_frame],
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
        arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), ns+'/'+base_link_frame, 'x500_d435_1/link/realsense_d435'],
        
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
    # drone_detection_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('d2dtracker_drone_detector'),
    #             'detection.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'detections_topic': 'detections_poses',
    #         'depth_topic' : 'interceptor/depth_image',
    #         'detector_ns': ''
    #     }.items()
    # )

    # Kalman filter
    file_name = 'kf_param.yaml'
    package_share_directory = get_package_share_directory('d2dtracker_sim')
    kf_file_path = os.path.join(package_share_directory, file_name)
    kf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('multi_target_kf'),
                'launch/kf_const_vel.launch.py'
            ])
        ]),
        launch_arguments={
            'detections_topic': 'yolo_detections_poses',
            'kf_ns' : '',
            'kf_yaml': kf_file_path
        }.items()
    )

    # Trajectory prediction
    predictor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('trajectory_prediction'),
                'launch/predictor.launch.py'
            ])
        ]),
        launch_arguments={
            'kf_topic': 'kf/good_tracks',
            'namespace' : '',
            'log_level' : 'info'
        }.items()
    )
    
    # YOLOv8
    yolov8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('yolov8_bringup'),
                'launch/yolov8.launch.py'
            ])
        ]),
        launch_arguments={
            'model': '/home/user/shared_volume/ros2_ws/src/d2dtracker_drone_detector/config/drone_detection_v3.pt',
            'threshold' : '0.5',
            'input_image_topic' : '/interceptor/image',
            'device': 'cuda:0'
        }.items()
    )

    # Yolo to pose node
    yolo2pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('d2dtracker_drone_detector'),
                'yolo2pose.launch.py'
            ])
        ]),
        launch_arguments={
            'depth_topic': 'interceptor/depth_image',
            'debug' : 'false',
            'caminfo_topic' : 'interceptor/camera_info',
            'detections_poses_topic': 'yolo_detections_poses',
            'yolo_detections_topic': 'detections',
            'detector_ns' : '',
            'reference_frame' : 'map'
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
    # ld.add_action(xrce_agent_launch)
    # ld.add_action(drone_detection_launch)
    ld.add_action(kf_launch)
    # ld.add_action(px4_ros_node)
    ld.add_action(predictor_launch)
    ld.add_action(yolov8_launch)
    ld.add_action(yolo2pose_launch)
    ld.add_action(mavros_launch)

    return ld