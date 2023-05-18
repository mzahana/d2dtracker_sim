import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand


class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_contorl')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.status_sub_ = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status',
            self.vehicleStatusCallback,
            qos_profile)
        
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
        self.theta_ = 0.0

        self.declare_parameter('system_id', 1)
        self.sys_id_ = self.get_parameter('system_id').get_parameter_value().integer_value
        self.declare_parameter('circle_radius', 5.0)
        self.radius_ = self.get_parameter('circle_radius').get_parameter_value().double_value
        self.declare_parameter('circle_omega', 0.5)
        self.omega_ = self.get_parameter('circle_omega').get_parameter_value().double_value

        self.declare_parameter('offboard_altitude', 5.0)
        self.offboard_altitude_ = self.get_parameter('offboard_altitude').get_parameter_value().double_value
        if self.offboard_altitude_ > 0:
            self.offboard_altitude_ = -1.0*self.offboard_altitude_
 
    def vehicleStatusCallback(self, msg: VehicleStatus):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state_ = msg.nav_state
        if msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.is_armed_ = True
        else:
            self.is_armed_ = False

    # def offbTimerCallback(self):
    #     if (self.offboard_setpoint_counter_ == 10):
    #         # Change to Offboard mode after 10 setpoints
    #         self.publishVehicleCommand(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
    #         # Arm the vehicle
    #         self.arm()

    #     # Offboard_control_mode needs to be paired with trajectory_setpoint
    #     self.publishOffboardControlMode()
    #     self.publish_trajectory_setpoint()

    #     # stop the counter after reaching 11
    #     if (self.offboard_setpoint_counter_ < 11):
    #         self.offboard_setpoint_counter_ += 1

    # Arm the vehicle
    def arm(self):
        self.publishVehicleCommand(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send to system {}".format(self.sys_id_))

    # Disarm the vehicle
    def disarm(self):
        self.publishVehicleCommand(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

    
    def publishVehicleCommand(self, command, param1=0.0, param2=0.0):
        '''
        Publish vehicle commands
        command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
        param1    Command parameter 1 as defined by MAVLink uint16 VEHICLE_CMD enum
        param2    Command parameter 2 as defined by MAVLink uint16 VEHICLE_CMD enum
        '''
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

        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position[0] = self.radius_ * np.cos(self.theta_)
        trajectory_msg.position[1] = self.radius_ * np.sin(self.theta_)
        trajectory_msg.position[2] = self.offboard_altitude_
        self.publisher_trajectory_.publish(trajectory_msg)

        self.theta_ = self.theta_ + self.omega_ * self.dt_

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