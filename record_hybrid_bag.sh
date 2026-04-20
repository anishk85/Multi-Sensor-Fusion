#!/bin/bash
# Script to record raw, filtered, and EKF odometry for moving bot analysis

OUTPUT_DIR="/home/rosdata/rosbags"
mkdir -p $OUTPUT_DIR
BAG_NAME="${OUTPUT_DIR}/moving_bot_live_test_$(date +%Y%m%d_%H%M%S)"

echo "Recording ROS bag to ${BAG_NAME}..."
echo "Press Ctrl+C to stop recording."

ros2 bag record -o $BAG_NAME \
    /clock \
    /tf \
    /tf_static \
    /cmd_vel \
    /joint_states \
    /imu/data \
    /imu/data/filtered \
    /gps/fix \
    /gps/fix/filtered \
    /diff_drive_controller/odom \
    /diff_drive_controller/odom/filtered \
    /odometry/visual \
    /odometry/visual/filtered \
    /odometry/lidar \
    /odometry/lidar/filtered \
    /odometry/local \
    /odometry/global \
    /ground_truth/odom
