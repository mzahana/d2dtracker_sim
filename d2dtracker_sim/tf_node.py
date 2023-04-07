import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import tf2_ros
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from math import sqrt
import numpy as np

def quaternion_multiply(q1, q2):
    # q=[w,x,y,z]
    return (
        q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3],
        q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2],
        q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1],
        q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0])

def ned2enu(q_ned):
    """
    Converts a quaternion expressed in NED frame to a quaternion expressed in ENU frame

    :param q_ned: quaternion expressed in NED frame as a numpy array of shape (4,). q=[w,x,y,z]
    :return: quaternion expressed in ENU frame as a numpy array of shape (4,)
    """

    R_ned2enu = np.array([[0, 1, 0, 0],
                          [1, 0, 0, 0],
                          [0, 0, -1, 0],
                          [0, 0, 0, -1]])

    q_enu = np.dot(R_ned2enu, q_ned)

    return q_enu

class PX4TF(Node):
    def __init__(self):
        super().__init__('px4_tf_node')
        self._odom_msg = Odometry()
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._ns = self.get_namespace()

        self.declare_parameter('parent_frame', 'world')
        self._parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        
        self.declare_parameter('child_frame', 'base_link')
        self._child_frame = self.get_parameter('child_frame').get_parameter_value().string_value

        ## Configure subscriptions
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self._odometry_sub = self.create_subscription(
            VehicleOdometry, 
            'fmu/out/vehicle_odometry', 
            self.odomCallback, 
            qos_profile
        )

        self._odometry_pub = self.create_publisher(Odometry, 'odometry', 10)
        self._pose_pub = self.create_publisher(PoseStamped, 'pose', 10)
        timer_period = 0.05  # seconds
        self._tf_timer = self.create_timer(timer_period, self.tfPubCallback)

    def odomCallback(self, msg):
        pose_msg = PoseStamped()

        self._odom_msg.header.stamp = self.get_clock().now().to_msg()
        self._odom_msg.header.frame_id = self._ns+'/'+self._parent_frame
        self._odom_msg.child_frame_id = self._ns+'/'+self._child_frame

        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self._ns+'/'+self._parent_frame

        # Set position
        self._odom_msg.pose.pose.position.x = float(msg.position[0])
        self._odom_msg.pose.pose.position.y = -float(msg.position[1])
        self._odom_msg.pose.pose.position.z = -float(msg.position[2])

        pose_msg.pose.position.x = float(msg.position[0])
        pose_msg.pose.position.y = -float(msg.position[1])
        pose_msg.pose.position.z = -float(msg.position[2])

        # Set linear velocity
        self._odom_msg.twist.twist.linear.x = float(msg.velocity[0])
        self._odom_msg.twist.twist.linear.y = -float(msg.velocity[1])
        self._odom_msg.twist.twist.linear.z = -float(msg.velocity[2])

        # Set orientation
        # q1 = Quaternion(w = 0.0, x = sqrt(2)/2, y = sqrt(2)/2, z = 0.0)
        # q2 = Quaternion(w = float(msg.q[0]), x = float(msg.q[1]), y = float(msg.q[2]), z = float(msg.q[3]))
        # q = tf2_ros.transformations.quaternion_multiply(q1, q2)

        # q_ned=(msg.q[0],
        #        msg.q[1],
        #        msg.q[2],
        #        msg.q[3])
        # q_rot=(0.0, sqrt(2.)/2., sqrt(2.)/2., -1.0)
        # q_enu = quaternion_multiply(q_rot, q_ned)

        # q_ned=np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])
        # q_enu = ned2enu(q_ned)

        self._odom_msg.pose.pose.orientation.w = float(msg.q[0])
        self._odom_msg.pose.pose.orientation.x = float(msg.q[1])
        self._odom_msg.pose.pose.orientation.y = -float(msg.q[2])
        self._odom_msg.pose.pose.orientation.z = -float(msg.q[3])

        pose_msg.pose.orientation.w = float(msg.q[0])
        pose_msg.pose.orientation.x = float(msg.q[1])
        pose_msg.pose.orientation.y = -float(msg.q[2])
        pose_msg.pose.orientation.z = -float(msg.q[3])

        # self._odom_msg.pose.pose.orientation.w = q.w
        # self._odom_msg.pose.pose.orientation.x = q.x
        # self._odom_msg.pose.pose.orientation.y = q.y
        # self._odom_msg.pose.pose.orientation.z = q.z

        # Publish odometry message
        self._odometry_pub.publish(self._odom_msg)
        self._pose_pub.publish(pose_msg)

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


