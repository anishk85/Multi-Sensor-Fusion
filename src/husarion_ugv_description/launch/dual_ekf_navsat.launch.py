#!/usr/bin/env python3
# =============================================================================
#  Dual EKF + NavSat Fusion Launch File for Husarion Lynx
#
#  This launch file brings up:
#    1. Gazebo sim + robot + bridges + controllers      (includes gz_lynx.launch.py)
#    2. EKF #1 — local odom filter    (odom → base_link)
#    3. EKF #2 — global map filter    (map → odom, fuses GPS)
#    4. navsat_transform_node          (GPS lat/lon → local x,y)
#
#  Usage:
#    ros2 launch husarion_ugv_description dual_ekf_navsat.launch.py
#    ros2 launch husarion_ugv_description dual_ekf_navsat.launch.py world:=/path/to/world.sdf
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

    # ---- Include the base Gazebo launch (sim + robot + bridges + controllers) ----
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

    # ---- EKF #1: Local Odometry Filter ----
    # Publishes: odom → base_link (smooth, no GPS jumps)
    # Fuses: wheel odom velocities + IMU
    # Delayed 8s to let controllers start first (they start at 5-6s)
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

    # ---- EKF #2: Global Map Filter ----
    # Publishes: map → odom (globally corrected using GPS)
    # Fuses: wheel odom velocities + IMU + GPS position from navsat
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

    # ---- NavSat Transform ----
    # Converts GPS lat/lon → local XY odometry
    # Subscribes to: /gps/fix, /imu/data, /odometry/global (EKF #2 output)
    # Publishes: /odometry/gps (fed to EKF #2)
    # Delayed 11s to let EKFs initialize first (handles the "circular" dependency)
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


