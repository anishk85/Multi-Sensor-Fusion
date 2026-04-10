#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("husarion_ugv_description")

    urdf_path = os.path.join(pkg, "urdf", "lynx_simple.urdf")
    rviz_config = os.path.join(pkg, "rviz", "lynx_simple.rviz")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time"
        ),

        # Publishes the robot URDF and broadcasts the TF tree
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),

        # GUI with sliders to manually set wheel joint positions
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),

        # RViz with a pre-built config showing the robot model and TF
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
