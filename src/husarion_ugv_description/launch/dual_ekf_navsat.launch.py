#!/usr/bin/env python3
# =============================================================================
#  Dual launch
#
#  start:
#    1. gazebo
#    2. ekf 1 local
#    3. ekf 2 map
#    4. navsat
# =============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("husarion_ugv_description")

    ekf_config = os.path.join(pkg, "config", "dual_ekf_navsat.yaml")

    # start gazebo
    gz_lynx = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "gz_lynx.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "spawn_z": LaunchConfiguration("spawn_z"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    # EKF 1: local
    # wait 8s for controller to start.
    ekf_odom = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[ekf_config, {"use_sim_time": True}],
                remappings=[
                    ("odometry/filtered", "odometry/local"),
                ],
            )
        ],
    )

    # EKF 2: map
    ekf_map = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[ekf_config, {"use_sim_time": True}],
                remappings=[
                    ("odometry/filtered", "odometry/global"),
                ],
            )
        ],
    )

    # navsat
    # wait 11s for EKF to think.
    navsat = TimerAction(
        period=11.0,
        actions=[
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[ekf_config, {"use_sim_time": True}],
                remappings=[
                    ("imu", "imu/data"),
                    ("gps/fix", "gps/fix"),
                    ("odometry/filtered", "odometry/global"),
                ],
            )
        ],
    )

    return LaunchDescription([
        # --- Launch arguments (passed through to gz_lynx) ---
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(pkg, "worlds", "agriculture.world"),
            description="Path to Gazebo world file (default: agriculture.world with GPS spherical coordinates)",
        ),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.35",
            description="Robot spawn height",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz",
        ),

        # --- Bring up everything ---
        gz_lynx,       # Gazebo + robot + bridges + controllers (t=0-6s)
        ekf_odom,      # EKF #1: odom→base_link (t=8s)
        ekf_map,       # EKF #2: map→odom (t=8s)
        navsat,        # NavSat transform (t=11s, after EKFs are up)
    ])


