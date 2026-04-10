#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock"
    )

    # Enable / disable each odometry source independently
    use_visual_odom_arg = DeclareLaunchArgument(
        "use_visual_odom", default_value="true", description="Launch RTAB-Map RGB-D odometry"
    )
    use_lidar_odom_arg = DeclareLaunchArgument(
        "use_lidar_odom", default_value="true", description="Launch LiDAR ICP odometry"
    )
    use_rtabmap_mapping_arg = DeclareLaunchArgument(
        "use_rtabmap_mapping", default_value="false", description="Launch RTAB-Map SLAM mapping/database node"
    )

    # Common output topics for EKF inputs
    visual_odom_topic_arg = DeclareLaunchArgument(
        "visual_odom_topic", default_value="/odometry/visual", description="Visual odometry output topic"
    )
    lidar_odom_topic_arg = DeclareLaunchArgument(
        "lidar_odom_topic", default_value="/odometry/lidar", description="LiDAR odometry output topic"
    )

    # Sensor input topics
    rgb_topic_arg = DeclareLaunchArgument("rgb_topic", default_value="/realsense/image")
    depth_topic_arg = DeclareLaunchArgument("depth_topic", default_value="/realsense/depth_image")
    camera_info_topic_arg = DeclareLaunchArgument("camera_info_topic", default_value="/realsense/camera_info")
    imu_topic_arg = DeclareLaunchArgument("imu_topic", default_value="/imu/data")
    lidar_points_topic_arg = DeclareLaunchArgument("lidar_points_topic", default_value="/lidar/points")

    base_frame_arg = DeclareLaunchArgument(
        "base_frame", default_value="base_link", description="Robot base frame for visual odometry"
    )
    rtabmap_database_path_arg = DeclareLaunchArgument(
        "rtabmap_database_path",
        default_value="/home/rosdata/rtabmap/rtabmap.db",
        description="Path to RTAB-Map database (.db)",
    )

    # Visual odometry (RGB-D) via RTAB-Map
    visual_odom_node = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="visual_odometry",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_visual_odom")),
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "frame_id": LaunchConfiguration("base_frame"),
                "publish_tf": False,
                "subscribe_depth": True,
                "subscribe_rgbd": False,
                "approx_sync": True,
                "wait_imu_to_init": False,
                # --- VO tuning for mixed ground + horizon features ---
                # Use GFTT features (0) — better corner detection on tiles
                "Vis/FeatureType": "6",
                "Vis/MaxFeatures": "1000",
                # PnP estimation (1) works with coplanar features (ground)
                # Default (0) = 3D-3D which fails when features are on a plane
                "Vis/EstimationType": "1",
                "Vis/MinInliers": "8",
                "OdomF2M/MaxSize": "3000",
                "OdomF2M/MaxNewFeatures": "300",
                "Odom/ResetCountdown": "2",
            }
        ],
        remappings=[
            ("rgb/image", LaunchConfiguration("rgb_topic")),
            ("depth/image", LaunchConfiguration("depth_topic")),
            ("rgb/camera_info", LaunchConfiguration("camera_info_topic")),
            ("imu", LaunchConfiguration("imu_topic")),
            ("odom", LaunchConfiguration("visual_odom_topic")),
        ],
    )

    # LiDAR odometry via RTAB-Map ICP
    lidar_odom_node = Node(
        package="rtabmap_odom",
        executable="icp_odometry",
        name="lidar_odometry",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_lidar_odom")),
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "frame_id": LaunchConfiguration("base_frame"),
                "publish_tf": False,
                "wait_imu_to_init": False,
                "deskewing": False,
                # RTAB-Map internal parameters are strings:
                "Icp/PointToPlane": "true",
                "Icp/Iterations": "10",
                "Icp/VoxelSize": "0.15",
                "Icp/Epsilon": "0.001",
                "Icp/MaxTranslation": "1.0",
                "Icp/MaxCorrespondenceDistance": "1.0",
                "Icp/OutlierRatio": "0.7",
                "Odom/ScanKeyFrameThr": "0.7",
            },
        ],
        remappings=[
            ("scan_cloud", LaunchConfiguration("lidar_points_topic")),
            ("imu", LaunchConfiguration("imu_topic")),
            ("odom", LaunchConfiguration("lidar_odom_topic")),
        ],
    )

    # Optional mapping/database node (can be opened later with rtabmap-databaseViewer)
    rtabmap_mapping_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rtabmap_mapping")),
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "frame_id": LaunchConfiguration("base_frame"),
                "publish_tf": False,
                "subscribe_rgb": True,
                "subscribe_depth": True,
                "subscribe_scan_cloud": True,
                "subscribe_odom_info": False,
                "approx_sync": True,
                "database_path": LaunchConfiguration("rtabmap_database_path"),
                # RTAB-Map internal parameters are strings:
                "Mem/IncrementalMemory": "true",
                "Mem/InitWMWithAllNodes": "false",
                "RGBD/CreateOccupancyGrid": "false",
            }
        ],
        remappings=[
            ("rgb/image", LaunchConfiguration("rgb_topic")),
            ("depth/image", LaunchConfiguration("depth_topic")),
            ("rgb/camera_info", LaunchConfiguration("camera_info_topic")),
            ("scan_cloud", LaunchConfiguration("lidar_points_topic")),
            ("odom", LaunchConfiguration("visual_odom_topic")),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_visual_odom_arg,
        use_lidar_odom_arg,
        use_rtabmap_mapping_arg,
        visual_odom_topic_arg,
        lidar_odom_topic_arg,
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        imu_topic_arg,
        lidar_points_topic_arg,
        base_frame_arg,
        rtabmap_database_path_arg,
        visual_odom_node,
        lidar_odom_node,
        rtabmap_mapping_node,
    ])
