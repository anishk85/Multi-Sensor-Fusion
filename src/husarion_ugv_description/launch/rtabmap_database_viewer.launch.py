#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    db_path_arg = DeclareLaunchArgument(
        "db_path",
        default_value="/home/rosdata/rtabmap/rtabmap.db",
        description="RTAB-Map database path (.db) to open in database viewer",
    )

    open_viewer = ExecuteProcess(
        cmd=["rtabmap-databaseViewer", LaunchConfiguration("db_path")],
        output="screen",
    )

    return LaunchDescription([
        db_path_arg,
        open_viewer,
    ])
