import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import tf2_ros
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from math import sqrt

class PX4TF(Node):
    def __init__(self):
        super().__init__('px4_tf_node')
        self._odom_msg = Odometry()
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._ns = self.get_namespace()

        self.declare_parameter('parent_frame', 'world')
        self._parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        
        self.declare_parameter('child_frame', 'world')
        self._child_frame = self.get_parameter('child_frame').get_parameter_value().string_value

        ## Configure subscriptions
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._odometry_sub = self.create_subscription(
            VehicleOdometry, 
            'fmu/out/vehicle_odometry', 
            self.odomCallback, 
            qos_profile
        )

        self._odometry_pub = self.create_publisher(Odometry, 'odometry', 10)
        timer_period = 0.05  # seconds
        self._tf_timer = self.create_timer(timer_period, self.tfPubCallback)

    def odomCallback(self, msg):
        # print(msg)
        self._odom_msg.header.stamp = self.get_clock().now().to_msg()
        self._odom_msg.header.frame_id = self._ns+'/'+self._parent_frame
        self._odom_msg.child_frame_id = self._ns+'/'+self._child_frame

        # Set position
        self._odom_msg.pose.pose.position.x = float(msg.position[1])
        self._odom_msg.pose.pose.position.y = float(msg.position[0])
        self._odom_msg.pose.pose.position.z = float(-msg.position[2])

        # Set orientation
        # q1 = Quaternion(w = 0.0, x = sqrt(2)/2, y = sqrt(2)/2, z = 0.0)
        # q2 = Quaternion(w = float(msg.q[0]), x = float(msg.q[1]), y = float(msg.q[2]), z = float(msg.q[3]))
        # q = tf2_ros.transformations.quaternion_multiply(q1, q2)
        self._odom_msg.pose.pose.orientation.w = float(msg.q[0])
        self._odom_msg.pose.pose.orientation.x = float(msg.q[2])
        self._odom_msg.pose.pose.orientation.y = float(msg.q[1])
        self._odom_msg.pose.pose.orientation.z = float(-msg.q[3])

        # self._odom_msg.pose.pose.orientation.w = q.w
        # self._odom_msg.pose.pose.orientation.x = q.x
        # self._odom_msg.pose.pose.orientation.y = q.y
        # self._odom_msg.pose.pose.orientation.z = q.z

        # Publish odometry message
        self._odometry_pub.publish(self._odom_msg)

    def tfPubCallback(self):
        # Calculate transform from pose_frame to base_frame based on vehicle pose
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._ns+'/'+self._parent_frame
        transform.child_frame_id = self._ns+'/'+self._child_frame

        transform.transform.translation.x =  self._odom_msg.pose.pose.position.x
        transform.transform.translation.y =  self._odom_msg.pose.pose.position.y
        transform.transform.translation.z =  self._odom_msg.pose.pose.position.z

        transform.transform.rotation.w = self._odom_msg.pose.pose.orientation.w
        transform.transform.rotation.x = self._odom_msg.pose.pose.orientation.x
        transform.transform.rotation.y = self._odom_msg.pose.pose.orientation.y
        transform.transform.rotation.z = self._odom_msg.pose.pose.orientation.z

         # Broadcast transform from pose_frame to base_frame	
        self._tf_broadcaster.sendTransform(transform)

def main(args=None):	
    rclpy.init(args=args)	
    px4_tf = PX4TF()	
    rclpy.spin(px4_tf)
    rclpy.shutdown()	
if __name__ == '__main__':	
    main()


