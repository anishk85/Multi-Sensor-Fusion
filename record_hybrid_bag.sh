#!/bin/bash
# Script to record sensory data and odometry for moving bot analysis

# Define the output directory and bag name
OUTPUT_DIR="/home/rosdata/rosbags"
mkdir -p $OUTPUT_DIR
BAG_NAME="${OUTPUT_DIR}/moving_bot_data_$(date +%Y%m%d_%H%M%S)"

echo "Recording ROS bag to ${BAG_NAME}..."
echo "Press Ctrl+C to stop recording."

ros2 bag record -o $BAG_NAME \
    /clock \
    /imu/data \
    /gps/fix \
    /diff_drive_controller/odom \
    /odometry/visual \
    /odometry/lidar \
    /odometry/filtered \
    /tf \
    /tf_static \
    /cmd_vel \
    /joint_states
