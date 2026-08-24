#!/usr/bin/env python3

"""
Full guidance + control test setup for the interceptor in PX4 SITL.

Brings up the whole chain, from the simulator to the attitude setpoints PX4 acts on:

    PX4 SITL (gz) -> MAVROS -> geometric_mavros_node -> geometric_controller/odom
                                                              |
      trajectory_publisher_node ---(TargetCommand)---> flight_manager_node
        (mav_navigator_ros: planner)                   (mav_navigator_ros: state machine)
                                                              |
                                                    geometric_controller/setpoint
                                                              |
                                                    geometric_controller_node
                                                              |
                                                    geometric_controller/cmd
                                                              |
                                                    geometric_mavros_node
                                                              |
                                              mavros/setpoint_raw/attitude -> PX4

The hard-coded static setpoint publisher this file used to run has been replaced by
the mav_navigator_ros pair: the planner generates a differentially flat reference
and the state machine gates it, so nothing reaches the controller until the vehicle
is armed, in OFFBOARD and holding a valid home pose.

Everything runs in the 'interceptor' namespace. Parameters come from
config/navigator/*.yaml in this package, keyed on the fully namespaced node name;
the launch arguments below override the handful worth changing per run.

A note on OFFBOARD bootstrap: PX4 refuses an OFFBOARD switch unless offboard
setpoints are already streaming, but the state machine is silent in READY. The
sim config therefore sets ready_setpoint_stream:=true, which streams a
hold-where-you-are setpoint while disarmed purely so the mode switch is accepted.
Both that and enable_sim_auto_arm are off by default in mav_navigator_ros and are
enabled here only because this is simulation.

Usage:
    # bring everything up, then drive it by hand
    ros2 launch d2dtracker_sim geometric_controller_test.launch.py

    ros2 topic echo /interceptor/flight_manager/state
    ros2 service call /interceptor/flight_manager/takeoff std_srvs/srv/Trigger {}
    ros2 service call /interceptor/trajectory_publisher/start std_srvs/srv/Trigger {}
    ros2 service call /interceptor/flight_manager/land std_srvs/srv/Trigger {}

    # hands free: auto takeoff and start the trajectory as soon as MISSION is reached
    ros2 launch d2dtracker_sim geometric_controller_test.launch.py \
        auto_takeoff:=true start_planner:=true

    # fly the figure of eight instead of a circle
    ros2 launch d2dtracker_sim geometric_controller_test.launch.py \
        trajectory_type:=2 radius:=3.0 omega:=0.4

    # watch it fly: reference path, flown trail, commanded vs actual heading
    ros2 launch d2dtracker_sim geometric_controller_test.launch.py \
        auto_takeoff:=true start_planner:=true rviz:=true

    # different world / spawn point
    ros2 launch d2dtracker_sim geometric_controller_test.launch.py \
        gz_world:=ihunter_world xpos:=-24.0 ypos:=8.0 zpos:=1.0
"""

import os
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, TimerAction,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def make_navigator_nodes(context, ns, sim_share):
    """Build the state machine and planner nodes.

    Done in an OpaqueFunction because the planner's 'origin' is a double array and
    the 'height' argument has to be substituted into one element of it, which needs
    the launch argument resolved to a real value first.
    """
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    height = float(arg('height'))

    flight_manager_yaml = os.path.join(sim_share, 'flight_manager.yaml')
    trajectory_publisher_yaml = os.path.join(sim_share, 'trajectory_publisher.yaml')

    flight_manager_node = Node(
        package='mav_navigator_ros',
        executable='flight_manager_node',
        name='flight_manager_node',
        namespace=ns,
        output='screen',
        parameters=[
            flight_manager_yaml,
            {
                'auto_takeoff': arg('auto_takeoff').lower() == 'true',
                'takeoff_altitude': float(arg('takeoff_altitude')),
                'enable_sim_auto_arm': arg('enable_sim_auto_arm').lower() == 'true',
            },
        ],
        remappings=[
            # the fused estimate geometric_mavros_node publishes, so the state
            # machine and the controller share one state estimate
            ('flight_manager/odom', 'geometric_controller/odom'),          # sub
            ('mavros/state', 'mavros/state'),                              # sub
            ('flight_manager/setpoint_in', 'flight_manager/setpoint_in'),  # sub
            # this is the only writer of the controller input
            ('flight_manager/setpoint_out', 'geometric_controller/setpoint'),  # pub
            ('flight_manager/state', 'flight_manager/state'),              # pub
            ('mavros/companion_process/status',
             'mavros/companion_process/status'),                           # pub
        ]
    )

    trajectory_publisher_node = Node(
        package='mav_navigator_ros',
        executable='trajectory_publisher_node',
        name='trajectory_publisher_node',
        namespace=ns,
        output='screen',
        parameters=[
            trajectory_publisher_yaml,
            {
                'trajectory_type': int(arg('trajectory_type')),
                'radius': float(arg('radius')),
                'omega': float(arg('omega')),
                'yaw_mode': int(arg('yaw_mode')),
                'origin': [0.0, 0.0, height],
                'start_on_launch': arg('start_planner').lower() == 'true',
                'frame_id': ns + '/odom',
            },
        ],
        remappings=[
            ('trajectory_publisher/odom', 'geometric_controller/odom'),    # sub
            ('mavros/state', 'mavros/state'),                              # sub
            ('trajectory_publisher/motion_selector',
             'trajectory_publisher/motion_selector'),                      # sub
            # into the state machine, never straight to the controller
            ('trajectory_publisher/setpoint', 'flight_manager/setpoint_in'),  # pub
            ('trajectory_publisher/flat_setpoint',
             'trajectory_publisher/flat_setpoint'),                        # pub
            ('trajectory_publisher/trajectory',
             'trajectory_publisher/trajectory'),                           # pub
        ]
    )

    return [flight_manager_node, trajectory_publisher_node]


def generate_launch_description():
    ld = LaunchDescription()

    ns = 'interceptor'
    sim_share = get_package_share_directory('d2dtracker_sim')

    # ---------------- launch arguments ----------------
    # simulator
    ld.add_action(DeclareLaunchArgument('gz_world', default_value='default',
                                        description='Gazebo world: default | ihunter_world | park'))
    ld.add_action(DeclareLaunchArgument('xpos', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('ypos', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('zpos', default_value='0.2'))

    # flight state machine
    ld.add_action(DeclareLaunchArgument('auto_takeoff', default_value='false',
                                        description='Climb as soon as armed + OFFBOARD, instead '
                                                    'of waiting for the takeoff service'))
    ld.add_action(DeclareLaunchArgument('takeoff_altitude', default_value='2.0',
                                        description='Metres above the latched home pose'))
    ld.add_action(DeclareLaunchArgument('enable_sim_auto_arm', default_value='true',
                                        description='Let the flight manager command OFFBOARD and '
                                                    'ARM by itself. Simulation only'))

    # planner
    ld.add_action(DeclareLaunchArgument('trajectory_type', default_value='1',
                                        description='0 STATIONARY, 1 CIRCLE, 2 LEMNISCATE, '
                                                    '3 POLYNOMIAL, 4 WAYPOINTS'))
    ld.add_action(DeclareLaunchArgument('radius', default_value='2.0',
                                        description='Shape radius in metres'))
    ld.add_action(DeclareLaunchArgument('omega', default_value='0.5',
                                        description='Shape angular rate in rad/s. Rejected at '
                                                    'startup if omega*radius exceeds max_velocity'))
    ld.add_action(DeclareLaunchArgument('height', default_value='2.0',
                                        description='Shape centre height, local ENU'))
    ld.add_action(DeclareLaunchArgument('yaw_mode', default_value='2',
                                        description='0 ZERO, 1 FIXED, 2 VELOCITY_ALIGNED, '
                                                    '3 POINT_AT'))
    ld.add_action(DeclareLaunchArgument('start_planner', default_value='false',
                                        description='Publish the reference immediately instead of '
                                                    'waiting for the start service'))

    # topology / timing
    ld.add_action(DeclareLaunchArgument('navigator_delay', default_value='12.0',
                                        description='Seconds to wait before starting the planner '
                                                    'and the state machine, so PX4, MAVROS and '
                                                    'the odometry chain are up first'))

    ld.add_action(DeclareLaunchArgument('rviz', default_value='false',
                                        description='Open RViz with the guidance/control layout: '
                                                    'reference path, flown trail, and commanded vs '
                                                    'actual heading arrows'))

    ld.add_action(DeclareLaunchArgument('use_navigator', default_value='true',
                                        description='Run the planner + state machine. Set false '
                                                    'when an external node owns '
                                                    'geometric_controller/setpoint'))

    xpos = LaunchConfiguration('xpos')
    ypos = LaunchConfiguration('ypos')
    zpos = LaunchConfiguration('zpos')

    # ---------------- PX4 SITL + Gazebo ----------------
    # Same model/airframe as interceptor.launch.py so the controller yaml
    # (mass, max_thrust) stays valid.
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('d2dtracker_sim'), 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_ns': ns,
            'headless': '0',
            'gz_world': LaunchConfiguration('gz_world'),
            'gz_model_name': 'x500_d435',
            'px4_autostart_id': '4020',
            'instance_id': '1',
            'xpos': xpos,
            'ypos': ypos,
            'zpos': zpos,
        }.items()
    )

    # /clock bridge. MAVROS is launched with use_sim_time:=true, so it needs this.
    clock_bridge = Node(
        package='ros_gz_bridge',
        name='ros_bridge_node_clock',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    )

    # ---------------- MAVROS ----------------
    plugins_file_path = os.path.join(sim_share, 'interceptor_px4_pluginlists.yaml')
    config_file_path = os.path.join(sim_share, 'interceptor_px4_config.yaml')
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('d2dtracker_sim'), 'mavros.launch.py'])
        ]),
        launch_arguments={
            'mavros_namespace': ns + '/mavros',
            'tgt_system': '2',
            'fcu_url': 'udp://:14541@127.0.0.1:14558',
            'pluginlists_yaml': plugins_file_path,
            'config_yaml': config_file_path,
            'base_link_frame': ns + '/base_link',
            'odom_frame': ns + '/odom',
            'map_frame': 'map'
        }.items()
    )

    # Static TF map -> interceptor/odom
    map2pose_tf_node = Node(
        package='tf2_ros',
        name='map2px4_' + ns + '_tf_node',
        executable='static_transform_publisher',
        arguments=[xpos, ypos, zpos, '0.0', '0.0', '0.0', 'map', ns + '/odom'],
    )

    # ---------------- geometric controller (core) ----------------
    geometric_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('mav_controllers_ros'),
                                  'launch/geometric_controller.launch.py'])
        ]),
        launch_arguments={
            'yaml_path': os.path.join(sim_share, 'geometric_controller.yaml'),
            'controller_ns': ns
        }.items()
    )

    # ---------------- geometric controller -> MAVROS interface ----------------
    geometric_to_mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('mav_controllers_ros'),
                                  'launch/geometric_to_mavros.launch.py'])
        ]),
        launch_arguments={
            'param_file': os.path.join(sim_share, 'geometric_mavros.yaml'),
            'mavros_ns': ns
        }.items()
    )

    # ---------------- guidance: state machine + planner ----------------
    # Instantiated directly rather than through the mav_navigator_ros launch files so
    # the launch arguments above can override individual YAML parameters.
    # Relative topic names resolve inside the 'interceptor' namespace, so
    # 'mavros/state' becomes /interceptor/mavros/state and so on.
    #
    # Delayed so PX4, MAVROS and the odometry chain are up first. Harmless either
    # way: the state machine sits in WAITING_FOR_HOME until odometry is valid, and
    # the planner publishes nothing until started.
    navigator_group = TimerAction(
        period=LaunchConfiguration('navigator_delay'),
        actions=[
            OpaqueFunction(function=make_navigator_nodes,
                           args=[ns, sim_share])
        ],
        condition=IfCondition(LaunchConfiguration('use_navigator'))
    )

    # ---------------- visualisation ----------------
    # Off by default: this launch file is routinely run headless, and RViz is a
    # noticeable extra load on the same machine as Gazebo and PX4.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(sim_share, 'geometric_controller_test.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    ld.add_action(gz_launch)
    ld.add_action(clock_bridge)
    ld.add_action(map2pose_tf_node)
    ld.add_action(mavros_launch)
    ld.add_action(geometric_controller_launch)
    ld.add_action(geometric_to_mavros_launch)
    ld.add_action(navigator_group)
    ld.add_action(rviz_node)

    return ld
