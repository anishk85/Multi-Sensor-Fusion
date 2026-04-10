#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("husarion_ugv_description")
    default_params = os.path.join(pkg, "config", "xbox_teleop.yaml")

    joy_dev_arg = DeclareLaunchArgument(
        "joy_dev",
        default_value="/dev/input/js0",
        description="Joystick device path",
    )

    teleop_params_arg = DeclareLaunchArgument(
        "teleop_params",
        default_value=default_params,
        description="Path to teleop parameter YAML",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "dev": LaunchConfiguration("joy_dev"),
            "autorepeat_rate": 20.0,
            "deadzone": 0.05,
        }],
    )

    teleop_node = Node(
        package="husarion_ugv_description",
        executable="diff_drive_xbox_teleop.py",
        name="diff_drive_xbox_teleop",
        output="screen",
        parameters=[LaunchConfiguration("teleop_params")],
    )

    return LaunchDescription([
        joy_dev_arg,
        teleop_params_arg,
        joy_node,
        teleop_node,
    ])

