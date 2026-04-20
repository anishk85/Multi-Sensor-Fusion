#!/bin/bash
# Script to record RAW sensor data and baseline EKF outputs

OUTPUT_DIR="/home/rosdata/rosbags"
mkdir -p $OUTPUT_DIR
BAG_NAME="${OUTPUT_DIR}/moving_bot_baseline_test_$(date +%Y%m%d_%H%M%S)"

echo "Recording ROS bag to ${BAG_NAME}..."
echo "Press Ctrl+C to stop recording."

ros2 bag record -o $BAG_NAME \
    /clock \
    /tf \
    /tf_static \
    /cmd_vel \
    /joint_states \
    /imu/data \
    /gps/fix \
    /diff_drive_controller/odom \
    /odometry/visual \
    /odometry/lidar \
    /odometry/local \
    /odometry/global \
    /ground_truth/odom
