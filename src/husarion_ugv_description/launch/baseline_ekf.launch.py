#!/usr/bin/env python3
# =============================================================================
#  Baseline EKF Launch File (NO Quantum Filter)
#
#  This is the CONTROL GROUP launch — identical to hybrid_qf_ekf.launch.py
#  but WITHOUT the quantum_filter_node. All sensors feed raw data directly
#  into the EKF.
#
#  Usage:
#    ros2 launch husarion_ugv_description baseline_ekf.launch.py
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

    ekf_config = os.path.join(pkg, "config", "baseline_ekf.yaml")

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

    # NO quantum_filter_node — raw sensors go directly to EKF

    # ---- EKF #1: Local Odometry Filter (reads RAW topics) ----
    ekf_local = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_fusion",
                output="screen",
                parameters=[ekf_config, {"use_sim_time": True}],
                remappings=[
                    ("odometry/filtered", "odometry/local"),
                ],
            )
        ],
    )

    # ---- EKF #2: Global Map Filter (reads RAW topics) ----
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

    # ---- NavSat Transform (reads RAW /gps/fix and /imu/data) ----
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
                    ("imu", "imu/data"),
                    ("gps/fix", "gps/fix"),
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
        # NO quantum_filter_node — this is the baseline control group
        ekf_local,         # EKF #1: odom→base_link from RAW sensors (t=9s)
        ekf_map,           # EKF #2: map→odom from RAW sensors (t=9s)
        navsat,            # NavSat transform from RAW GPS (t=12s)
    ])
