#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
import csv

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.create_subscription(
            PoseStamped,
            '/target/mavros/local_position/pose',
            self.pose_callback,
            qos_profile_sensor_data)
        self.csv_file = '/home/user/shared_volume/gazebo_sim_joystick_1.csv'
        # Open the CSV file
        self.file = open(self.csv_file, 'w', newline='')
        self.csv_writer = csv.writer(self.file)
        # Write the header
        self.csv_writer.writerow(["timestamp", "tx", "ty", "tz"])
        self.get_logger().info('Subscriber has been started with SensorDataQoS.')

    def pose_callback(self, msg):
        # Extract the pose data
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        tx = msg.pose.position.x
        ty = msg.pose.position.y
        tz = msg.pose.position.z
        # Write to the CSV file
        self.csv_writer.writerow([timestamp, tx, ty, tz])
        self.get_logger().info(f'Pose written to {self.csv_file}: {timestamp}, {tx}, {ty}, {tz}')

    def __del__(self):
        self.file.close()

def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()

    try:
        rclpy.spin(pose_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        pose_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
