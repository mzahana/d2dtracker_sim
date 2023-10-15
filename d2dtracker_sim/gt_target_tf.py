#!/usr/bin/env python3

"""
@Description
This node publishes ground truth measurement of the target drone.
This can be used to test the Kalmn filter performance, as we all all the subsequent modules (prediciton and tracking)
Subscribes to TF tree, and finds the transformation from map frame to the target/base_link
Then publishes pose measurements as
    geometry_msgs/msg/PoseArray

Author: Mohamed Abdelkader
Copyright 2023
Contact: mohamedashraf123@gmail.com

"""
import rclpy
from rclpy.node import Node

from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import TransformStamped

class TFLookupNode(Node):

    def __init__(self):
        super().__init__('tf_lookup_node')
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frames', ['base_link'])

        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frames = self.get_parameter('child_frames').get_parameter_value().string_array_value

        self.publisher_ = self.create_publisher(PoseArray, 'pose_array', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        pose_array = PoseArray()
        pose_array.header.frame_id = self.parent_frame
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for child_frame in self.child_frames:
            try:
                transform = self.tf_buffer.lookup_transform(self.parent_frame, child_frame, rclpy.time.Time())
                pose = self.transform_to_pose(transform)
                pose_array.poses.append(pose)
            except Exception as e:
                self.get_logger().warn(f"Failed to get transform from {self.parent_frame} to {child_frame}: {str(e)}")

        self.publisher_.publish(pose_array)

    def transform_to_pose(self, transform: TransformStamped) -> Pose:
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose


def main(args=None):
    rclpy.init(args=args)

    node = TFLookupNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
