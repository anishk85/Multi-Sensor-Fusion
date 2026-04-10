#!/usr/bin/env python3
# =============================================================================
#  Hybrid QF + Dual EKF + NavSat Fusion Launch File
#
#  Same as dual_ekf_navsat.launch.py but adds a Quantum Filter node
#  that denoises sensor data BEFORE it reaches the EKFs.
#
#  Pipeline:
#    Raw sensors → [quantum_filter_node] → /filtered topics → [EKF] → fused pose
#
#  Usage:
#    ros2 launch husarion_ugv_description hybrid_qf_ekf.launch.py
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

    # Use the hybrid config that reads from /filtered topics
    ekf_config = os.path.join(pkg, "config", "hybrid_qf_ekf.yaml")

    # ---- Include the base Gazebo launch ----
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

    # ---- Odometry Sources (Visual + LiDAR) ----
    # Launches RTAB-Map RGB-D odometry and LiDAR ICP odometry.
    # Scheduled at t=5s — sensors need to be up but before QF (t=7s)
    odom_sources = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, "launch", "odom_sources.launch.py")
                ),
                launch_arguments={
                    "use_visual_odom": LaunchConfiguration("use_visual_odom"),
                    "use_lidar_odom": LaunchConfiguration("use_lidar_odom"),
                    "use_sim_time": "true",
                }.items(),
            )
        ],
    )

    # ---- Quantum Filter Node ----
    # Subscribes to raw sensor topics, applies QPSO wavelet denoising,
    # publishes on /filtered topics.
    # Delayed 7s to ensure sensors are publishing.
    quantum_filter = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="husarion_ugv_description",
                executable="quantum_filter_node.py",
                name="quantum_filter_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    # ---- EKF #1: Local Odometry Filter (reads /filtered topics) ----
    ekf_odom = TimerAction(
        period=9.0,
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

    # ---- EKF #2: Global Map Filter (reads /filtered topics) ----
    ekf_map = TimerAction(
        period=9.0,
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

    # ---- NavSat Transform (reads /gps/fix/filtered) ----
    navsat = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[ekf_config, {"use_sim_time": True}],
                remappings=[
                    ("imu", "imu/data/filtered"),
                    ("gps/fix", "gps/fix/filtered"),
                    ("odometry/filtered", "odometry/global"),
                ],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(pkg, "worlds", "agriculture.world"),
            description="Path to Gazebo world file",
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
        DeclareLaunchArgument(
            "use_visual_odom",
            default_value="true",
            description="Launch RTAB-Map RGB-D visual odometry",
        ),
        DeclareLaunchArgument(
            "use_lidar_odom",
            default_value="true",
            description="Launch RTAB-Map ICP lidar odometry",
        ),

        gz_lynx,           # Gazebo + robot + bridges + controllers (t=0-4s)
        odom_sources,      # Visual + LiDAR odometry sources (t=5s)
        quantum_filter,    # Quantum Filter: denoise sensors (t=7s)
        ekf_odom,          # EKF #1: odom→base_link from /filtered (t=9s)
        ekf_map,           # EKF #2: map→odom from /filtered (t=9s)
        navsat,            # NavSat transform from /filtered GPS (t=12s)
    ])
