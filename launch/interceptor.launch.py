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
    # world = {'gz_world': 'default'}
    world = {'gz_world': 'ihunter_world'}
    model_name = {'gz_model_name': 'x500_d435'}
    autostart_id = {'px4_autostart_id': '4020'}
    instance_id = {'instance_id': '1'}
    # for 'default' world
    # xpos = {'xpos': '0.0'}
    # ypos = {'ypos': '0.0'}
    # zpos = {'zpos': '0.1'}
    # For 'ihunter_world'
    xpos = {'xpos': '-24.0'}
    ypos = {'ypos': '8.0'}
    zpos = {'zpos': '1.0'}
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
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/d435/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '/d435/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '/d435/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                   '/d435/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                   '--ros-args', '-r', '/d435/depth_image:='+ns+'/depth_image',
                   '-r', '/d435/image:='+ns+'/image',
                   '-r', '/d435/points:='+ns+'/points',
                   '-r', '/d435/camera_info:='+ns+'/camera_info'
                   ],
    )

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
            'reference_frame' : 'interceptor/odom'
        }.items()
    )

    
    # drone_path_predictor_ros node (GRU based drone trajectory prediction)
    # The yaml file is inside drone_path_predictor_ros/config
    file_name = 'trajectory_predictor.yaml'
    package_share_directory = get_package_share_directory('d2dtracker_sim')
    traj_pred_file_path = os.path.join(package_share_directory, file_name)
    gru_traj_prediction_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('drone_path_predictor_ros'),
                'launch/trajectory_predictor.launch.py'
            ])
        ]),
        launch_arguments={
            'gru_namespace': ns,
            'param_file': traj_pred_file_path,
            'pose_topic' : '/kf/good_tracks_pose_array',
        }.items()
    )
    
    # trajectory_generation node (MPC with 12 states)
    # The yaml file is inside trajectory_generation/config
    file_name = 'mpc_12state.yaml'
    package_share_directory = get_package_share_directory('d2dtracker_sim')
    traj_gen_file_path = os.path.join(package_share_directory, file_name)
    mpc_traj_gen_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('trajectory_generation'),
                'launch/mpc_12state.launch.py'
            ])
        ]),
        launch_arguments={
            'mpc_namespace': ns,
            'yaml_path': traj_gen_file_path,
            'mpc_odom_topic' : 'mavros/local_position/odom',
            'mpc_imu_topic' : 'mavros/imu/data',
            'mpc_ref_path_topic': 'out/gru_predicted_path',
            'mpc_cmd_topic': 'geometric_controller/multi_dof_setpoint',
        }.items()
    )

    interceptor_offboard_control_node = Node(
        package='d2dtracker_sim',
        executable='interceptor_offboard_control',
        output='screen',
        name='interceptor_offboard_node',
        namespace=ns,
        remappings=[
            ('mavros/state', 'mavros/state'),
            ('mavros/local_position/odom', 'mavros/local_position/odom'),
            ('mavros/setpoint_raw/local', 'mavros/setpoint_raw/local'),
            ('mpc_tracker/command/trajectory', '/mpc_tracker/command/trajectory')
        ]
    )

    # geometric controller node
    file_name = 'geometric_controller.yaml'
    package_share_directory = get_package_share_directory('d2dtracker_sim')
    geometric_controller_file_path = os.path.join(package_share_directory, file_name)
    geometric_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mav_controllers_ros'),
                'launch/geometric_controller.launch.py'
            ])
        ]),
        launch_arguments={
            'yaml_path': geometric_controller_file_path,
            'controller_ns': ns
            }.items()
    )

    # Geometric controller mavros interface node
    file_name = 'geometric_mavros.yaml'
    package_share_directory = get_package_share_directory('d2dtracker_sim')
    geometric_mavros_file_path = os.path.join(package_share_directory, file_name)
    geometric_to_mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mav_controllers_ros'),
                'launch/geometric_to_mavros.launch.py'
            ])
        ]),
        launch_arguments={
            'param_file': geometric_mavros_file_path,
            'mavros_ns': ns
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
    ld.add_action(map2pose_tf_node)
    ld.add_action(cam_tf_node)
    ld.add_action(ros_gz_bridge)
    ld.add_action(kf_launch) # Estimates target's states based on position measurements( Reqiures yolov8_launch & yolo2pose_launch OR gt_target_tf)
    # ld.add_action(predictor_launch) # WARNING will be removed in the future
    ld.add_action(yolov8_launch)
    # ld.add_action(yolo2pose_launch) # Comment this if you want to use the target ground truth (gt_target_tf.launch.py)
    ld.add_action(mavros_launch)
    ld.add_action(geometric_controller_launch)
    ld.add_action(geometric_to_mavros_launch)
    ld.add_action(gru_traj_prediction_launch)
    ld.add_action(mpc_traj_gen_launch)
    ld.add_action(rviz_node)
    
    # ld.add_action(interceptor_offboard_control_node) # WARNING Not used

    return ld
