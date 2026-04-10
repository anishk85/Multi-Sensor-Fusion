#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("husarion_ugv_description")
    ros_gz_sim_pkg = get_package_share_directory("ros_gz_sim")

    urdf_path = os.path.join(pkg, "urdf", "lynx_simple.urdf")
    rviz_config = os.path.join(pkg, "rviz", "lynx_simple.rviz")
    default_world_path = os.path.join(pkg, "worlds", "agriculture.world")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world_path,
        description="Absolute path to the Gazebo world file",
    )
    spawn_z_arg = DeclareLaunchArgument(
        "spawn_z",
        default_value="0.90",
        description="Initial robot spawn height in meters",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz together with Gazebo",
    )

    resource_paths = f"{pkg}:{os.path.join(pkg, 'models')}:{os.path.join(pkg, 'worlds')}"
    gz_sim_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[resource_paths, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
    )
    gz_file_path = SetEnvironmentVariable(
        name="GZ_FILE_PATH",
        value=[resource_paths, ":", EnvironmentVariable("GZ_FILE_PATH", default_value="")],
    )

    # World includes sensor plugins (IMU, NavSat, Sensors with ogre2)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r ", LaunchConfiguration("world")],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Publishes /robot_description and the TF tree from the URDF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # Bridge Gazebo's clock to ROS so use_sim_time works correctly
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Bridge all sensor topics from Gazebo to ROS
    sensor_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="sensor_bridge",
        arguments=[
            # IMU
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            # GPS
            "/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
            # LiDAR — point cloud from the VLP-16-like gpu_lidar
            "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            # RealSense D435 — color, depth, point cloud, and camera info
            "/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/realsense/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/realsense/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        output="screen",
    )

    # Spawn the robot into Gazebo using the /robot_description topic
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "lynx",
            "-z", LaunchConfiguration("spawn_z"),
            "-allow_renaming", "true",
        ],
        output="screen",
    )

    # RViz integrated in the same launch, always on simulation time.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=(
            ["-d", rviz_config] if os.path.exists(rviz_config) else []
        ) + [
            "--ros-args",
            "--log-level", "rviz:=warn",
            "--log-level", "rviz2:=warn",
        ],
        parameters=[{"use_sim_time": True}],
    )

    # Activate joint_state_broadcaster first, then the drive controller.
    # The delay gives Gazebo time to fully load the gz_ros2_control plugin.
    joint_state_broadcaster = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "-c", "/controller_manager",
                    "--controller-manager-timeout", "120",
                    "--switch-timeout", "120",
                ],
                output="screen",
            )
        ],
    )

    diff_drive_controller = TimerAction(
        period=20.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "diff_drive_controller",
                    "-c", "/controller_manager",
                    "--controller-manager-timeout", "120",
                    "--switch-timeout", "120",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        world_arg,
        spawn_z_arg,
        use_rviz_arg,
        gz_sim_resource_path,
        gz_file_path,
        gz_sim,
        robot_state_publisher,
        clock_bridge,
        sensor_bridge,
        spawn_robot,
        joint_state_broadcaster,
        diff_drive_controller,
        rviz_node,
    ])
