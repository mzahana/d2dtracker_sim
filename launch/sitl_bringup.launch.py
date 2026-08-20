"""Minimal PX4 SITL bringup for geometric-controller testing/tuning.

Launches ONLY: Gazebo + PX4 SITL (x500_d435 in the empty default world), the
map->odom static TF, the /clock bridge, and mavros — the same wiring and
parameters as d2dtracker_sim's interceptor.launch.py, minus the
perception/prediction/MPC stack.

    ros2 launch geo_tuner sitl_bringup.launch.py                 # sim+px4+mavros
    ros2 launch geo_tuner sitl_bringup.launch.py with_controller:=true
    ros2 launch geo_tuner sitl_bringup.launch.py headless:=1

with_controller:=true additionally starts the geometric controller and its
mavros interface (same configs d2dtracker_sim uses), so the vehicle is
ready for OFFBOARD + `ros2 launch geo_tuner field_tune.launch.py ns:=interceptor`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

NS = "interceptor"
XPOS, YPOS, ZPOS = "0.0", "0.0", "0.1"  # spawn in the empty default world


def generate_launch_description():
    sim_share = get_package_share_directory("d2dtracker_sim")

    headless = LaunchConfiguration("headless")
    with_controller = LaunchConfiguration("with_controller")

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare("d2dtracker_sim"), "gz_sim.launch.py"])]),
        launch_arguments={
            "gz_ns": NS,
            "headless": headless,
            "gz_world": "default",
            "gz_model_name": "x500_d435",
            "px4_autostart_id": "4020",
            "instance_id": "1",
            "xpos": XPOS, "ypos": YPOS, "zpos": ZPOS,
        }.items(),
    )

    map2pose_tf_node = Node(
        package="tf2_ros",
        name="map2px4_" + NS + "_tf_node",
        executable="static_transform_publisher",
        arguments=[XPOS, YPOS, ZPOS, "0.0", "0", "0", "map", NS + "/odom"],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        name="ros_bridge_node_clock",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
    )

    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare("d2dtracker_sim"), "mavros.launch.py"])]),
        launch_arguments={
            "mavros_namespace": NS + "/mavros",
            "tgt_system": "2",
            "fcu_url": "udp://:14541@127.0.0.1:14558",
            "pluginlists_yaml": os.path.join(
                sim_share, "interceptor_px4_pluginlists.yaml"),
            "config_yaml": os.path.join(
                sim_share, "interceptor_px4_config.yaml"),
            "base_link_frame": NS + "/base_link",
            "odom_frame": NS + "/odom",
            "map_frame": "map",
        }.items(),
    )

    geometric_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare("mav_controllers_ros"),
             "launch/geometric_controller.launch.py"])]),
        launch_arguments={
            "yaml_path": os.path.join(sim_share, "geometric_controller.yaml"),
            "controller_ns": NS,
        }.items(),
        condition=IfCondition(with_controller),
    )

    geometric_to_mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare("mav_controllers_ros"),
             "launch/geometric_to_mavros.launch.py"])]),
        launch_arguments={
            "param_file": os.path.join(sim_share, "geometric_mavros.yaml"),
            "mavros_ns": NS,
        }.items(),
        condition=IfCondition(with_controller),
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="0"),
        DeclareLaunchArgument("with_controller", default_value="true"),
        gz_launch,
        map2pose_tf_node,
        clock_bridge,
        mavros_launch,
        geometric_controller_launch,
        geometric_to_mavros_launch,
    ])
