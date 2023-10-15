from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='d2dtracker_sim',
            executable='gt_target_tf',
            name='gt_target_tf_node',
            output='screen',
            remappings=[
                ('/pose_array', '/yolo_detections_poses')
            ],
            parameters=[
                {'parent_frame': 'interceptor/odom'},
                {'child_frames': ['target/base_link']}
            ]
        )
    ])
