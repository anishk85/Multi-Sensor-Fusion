#!/bin/bash
# Record stationary GPS pipeline for drift debugging.
# Capture every stage so notebook can find which node injects motion.
#
# Stages captured:
#   /gps/fix                       — raw Gazebo NavSat output
#   /gps/fix/covariance_fixed      — relay output (covariance injected)
#   /odometry/gps                  — navsat_transform output (lat/lon → XY)
#   /odometry/local                — EKF#1 fused (no GPS)
#   /odometry/global               — EKF#2 fused (with GPS)
#   /ground_truth/odom             — Gazebo true pose (zero noise)
#
# Use case: launch baseline_ekf.launch.py, do NOT move the robot. Record 60-120s.
# Then run notebooks/pipeline_drift_debug.ipynb on the resulting bag.

OUTPUT_DIR="/home/rosdata/rosbags"
mkdir -p $OUTPUT_DIR
BAG_NAME="${OUTPUT_DIR}/stationary_pipeline_$(date +%Y%m%d_%H%M%S)"

echo "Recording stationary pipeline bag to ${BAG_NAME}..."
echo "DO NOT MOVE THE ROBOT. Let it sit for 60-120s."
echo "Press Ctrl+C to stop."

ros2 bag record -o $BAG_NAME \
    /clock \
    /tf \
    /tf_static \
    /imu/data \
    /imu/data/covariance_fixed \
    /gps/fix \
    /gps/fix/covariance_fixed \
    /odometry/gps \
    /diff_drive_controller/odom \
    /odometry/visual \
    /odometry/lidar \
    /odometry/local \
    /odometry/global \
    /ground_truth/odom \
    /cmd_vel \
    /joint_states
