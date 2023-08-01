import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry

from .trajectories import Circle3D, Infinity3D

from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_contorl')

        self.declare_parameter('system_id', 1)
        self.sys_id_ = self.get_parameter('system_id').get_parameter_value().integer_value

        self.declare_parameter('radius', 5.0)
        self.radius_ = self.get_parameter('radius').get_parameter_value().double_value

        self.declare_parameter('omega', 0.5)
        self.omega_ = self.get_parameter('omega').get_parameter_value().double_value

        self.declare_parameter('trajectory_type', 'circle')
        self.trajectory_type_ = self.get_parameter('trajectory_type').get_parameter_value().string_value

        self.declare_parameter('normal_vector', [0., 0., 1.])
        self.normal_vector_ = self.get_parameter('normal_vector').get_parameter_value().double_array_value

        self.declare_parameter('center', [0., 0., 1.])
        self.center_ = self.get_parameter('center').get_parameter_value().double_array_value

        # Initialize the trajectory generator based on the selected type
        if self.trajectory_type_== 'circle':
            self.trajectory_generator_ = Circle3D(np.array(self.normal_vector_), np.array(self.center_), radius=self.radius_, omega=self.omega_)
        elif self.trajectory_type_ == 'infty':
            self.trajectory_generator_ = Infinity3D(np.array(self.normal_vector_), np.array([10, 0, 5]), radius=self.radius_, omega=self.omega_)
        else:
            raise ValueError("Invalid trajectory type. Supported types are 'circle', 'infty'.")
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        qos_profile_volatile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.status_sub_ = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status',
            self.vehicleStatusCallback,
            qos_profile)
        
        self.odom_sub_ = self.create_subscription(
            Odometry,
            'px4_ros/odom',
            self.odomCallback,
            qos_profile_volatile)
        
        self.vehicle_path_pub_ = self.create_publisher(Path, 'offboard_visualizer/vehicle_path', 10)
        self.setpoint_path_pub_ = self.create_publisher(Path, 'offboard_visualizer/setpoint_path', 10)
        
        self.odom_ = Odometry()

        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.publisher_offboard_mode_ = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory_ = self.create_publisher(TrajectorySetpoint, 'fmu/in/trajectory_setpoint', qos_profile)

        timer_period = 0.02  # seconds
        self.cmd_timer_ = self.create_timer(timer_period, self.cmdloopCallback)

        # timer_period = 0.1  # 100 milliseconds
        # self.offboard_mode_timer_ = self.create_timer(timer_period, self.offbTimerCallback)

        self.offboard_setpoint_counter_ = 0

        self.nav_state_ = VehicleStatus.NAVIGATION_STATE_MAX
        self.is_armed_ = False
        self.dt_ = timer_period

        self.vehicle_path_msg_ = Path()
        self.setpoint_path_msg_ = Path()


    def vehicleStatusCallback(self, msg: VehicleStatus):
        # TODO: handle NED->ENU transformation
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state_ = msg.nav_state
        if msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.is_armed_ = True
        else:
            self.is_armed_ = False

    def odomCallback(self, msg: Odometry):
        self.odom_ = msg

    def create_arrow_marker(self, id, tail, vector):
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = 'map'
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

    # Arm the vehicle
    def arm(self):
        self.publishVehicleCommand(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send to system {}".format(self.sys_id_))

    # Disarm the vehicle
    def disarm(self):
        self.publishVehicleCommand(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")


    def publishVehicleCommand(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command  # command ID
        msg.target_system = self.sys_id_  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)


    def publishOffboardControlMode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.publisher_offboard_mode_.publish(msg)

    def cmdloopCallback(self):
        if (self.offboard_setpoint_counter_ == 60):
            # Change to Offboard mode after 10 setpoints
            self.publishVehicleCommand(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # Arm the vehicle
            self.arm()

        # Publish offboard control modes
        self.publishOffboardControlMode()

        point = self.trajectory_generator_.generate_trajectory_setpoint(Clock().now().nanoseconds / 1000/1000/1000)

        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position[0] = point[1]
        trajectory_msg.position[1] = point[0]
        trajectory_msg.position[2] = -point[2]

        self.publisher_trajectory_.publish(trajectory_msg)
        
         # Publish time history of the vehicle path
        vehicle_pose_msg = PoseStamped()
        vehicle_pose_msg.header = self.odom_.header
        vehicle_pose_msg.pose.position = self.odom_.pose.pose.position
        vehicle_pose_msg.pose.orientation = self.odom_.pose.pose.orientation
        self.vehicle_path_msg_.header = self.odom_.header
        self.vehicle_path_msg_.poses.append(vehicle_pose_msg)
        if (len(self.vehicle_path_msg_.poses) > 500):
            self.vehicle_path_msg_.poses.pop(0)

        self.vehicle_path_pub_.publish(self.vehicle_path_msg_)

        # Publish time history of the vehicle path
        # setpoint_pose_msg = vector2PoseMsg('map', self.setpoint_position, self.vehicle_attitude)
        setpoint_pose_msg = PoseStamped()
        setpoint_pose_msg.header.frame_id = self.odom_.header.frame_id
        setpoint_pose_msg.header.stamp = self.get_clock().now().to_msg()
        setpoint_pose_msg.pose.position.x = point[0]
        setpoint_pose_msg.pose.position.y = point[1]
        setpoint_pose_msg.pose.position.z = point[2]
        self.setpoint_path_msg_.header = setpoint_pose_msg.header
        self.setpoint_path_msg_.poses.append(setpoint_pose_msg)
        if (len(self.setpoint_path_msg_.poses) > 500):
            self.setpoint_path_msg_.poses.pop(0)

        self.setpoint_path_pub_.publish(self.setpoint_path_msg_)

        # stop the counter after reaching 11
        if (self.offboard_setpoint_counter_ < 61):
            self.offboard_setpoint_counter_ += 1


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()