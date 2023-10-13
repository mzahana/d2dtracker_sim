import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from mavros_msgs.msg import State, PositionTarget
from trajectory_msgs.msg import MultiDOFJointTrajectory


class OffboardControl(Node):

    def __init__(self):
        super().__init__('interceptor_offboard_contorl_node')

        self.is_armed_ = False

        self.odom_received_ = False

        self.odom_msg_ = Odometry() # latest odom
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_volatile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.status_sub_ = self.create_subscription(
            State,
            'mavros/state',
            self.vehicleStatusCallback,
            qos_profile_transient)
        
        self.odom_sub_ = self.create_subscription(
            Odometry,
            'mavros/local_position/odom',
            self.odomCallback,
            qos_profile_sensor_data)
        
        self.multidof_sub_ = self.create_subscription(
            MultiDOFJointTrajectory,
            'mpc_tracker/command/trajectory',
            self.multiDofCallback,
            10)
        

        self.setopint_pub_ = self.create_publisher(PositionTarget, 'mavros/setpoint_raw/local', qos_profile_sensor_data)


    def vehicleStatusCallback(self, msg: State):
        self.is_armed_ = msg.armed

    def odomCallback(self, msg: Odometry):
        self.odom_msg_ = msg
        self.odom_received_ = True

    def multiDofCallback(self, msg: MultiDOFJointTrajectory):
        if (not self.odom_received_):
            return
        
        if (len(msg.points) < 1):
            return
        
        x = msg.points[0].transforms[0].translation.x
        y = msg.points[0].transforms[0].translation.y
        z = msg.points[0].transforms[0].translation.z
        vx = msg.points[0].velocities[0].linear.x
        vy = msg.points[0].velocities[0].linear.y
        vz = msg.points[0].velocities[0].linear.z
        ax = msg.points[0].accelerations[0].linear.x
        ay = msg.points[0].accelerations[0].linear.y
        az = msg.points[0].accelerations[0].linear.z

        setpoint_msg = PositionTarget()
        setpoint_msg.header.stamp = self.get_clock().now().to_msg()
        setpoint_msg.header.frame_id = self.odom_msg_.header.frame_id
        setpoint_msg.coordinate_frame= PositionTarget.FRAME_LOCAL_NED
        pos_yaw_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                    PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ+ PositionTarget.IGNORE_YAW_RATE
        vel_yaw_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                    PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ+ PositionTarget.IGNORE_YAW_RATE
        setpoint_msg.type_mask = vel_yaw_mask
        setpoint_msg.position.x = x
        setpoint_msg.position.y = y
        setpoint_msg.position.z = z
        setpoint_msg.velocity.x = vx
        setpoint_msg.velocity.y = vy
        setpoint_msg.velocity.z = vz
        # yaw  = atan(d_y, d_x)
        yaw = np.arctan2(vy, vx)
        setpoint_msg.yaw = yaw

        self.setopint_pub_.publish(setpoint_msg)

    def create_arrow_marker(self, id, tail, vector):
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = self.odom_msg_.header.frame_id
        # msg.header.stamp = Clock().now().nanoseconds / 1000
        msg.ns = 'arrow'
        msg.id = id
        msg.type = Marker.ARROW
        msg.scale.x = 0.1
        msg.scale.y = 0.2
        msg.scale.z = 0.0
        msg.color.r = 0.5
        msg.color.g = 0.5
        msg.color.b = 0.0
        msg.color.a = 1.0
        dt = 0.3
        tail_point = Point()
        tail_point.x = tail[0]
        tail_point.y = tail[1]
        tail_point.z = tail[2]
        head_point = Point()
        head_point.x = tail[0] + dt * vector[0]
        head_point.y = tail[1] + dt * vector[1]
        head_point.z = tail[2] + dt * vector[2]
        msg.points = [tail_point, head_point]
        return msg


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()