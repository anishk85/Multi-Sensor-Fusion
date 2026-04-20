import glob
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry

plt.style.use('dark_background')
plt.rcParams['figure.figsize'] = (18, 16)
plt.rcParams['lines.linewidth'] = 2.0

def get_latest_bag(bag_prefix="/home/rosdata/rosbags/moving_bot_live_test_"):
    bags = sorted(glob.glob(f"{bag_prefix}*"))
    if not bags:
        raise ValueError("No rosbags found!")
    return bags[-1]

BAG_PATH = get_latest_bag()
print(f"Analyzing Bag: {BAG_PATH}")

def load_bag(bag_path):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    TOPICS = {
        '/diff_drive_controller/odom': ('raw_wheel', Odometry),
        '/diff_drive_controller/odom/filtered': ('rqnn_wheel', Odometry),
        '/odometry/visual': ('raw_visual', Odometry),
        '/odometry/visual/filtered': ('rqnn_visual', Odometry),
        '/odometry/lidar': ('raw_lidar', Odometry),
        '/odometry/lidar/filtered': ('rqnn_lidar', Odometry),
        '/imu/data': ('raw_imu', Imu),
        '/imu/data/filtered': ('rqnn_imu', Imu)
    }

    data = {k: [] for k, _ in TOPICS.values()}

    while reader.has_next():
        topic, msg_data, timestamp = reader.read_next()
        if topic in TOPICS:
            name, msg_type = TOPICS[topic]
            msg = deserialize_message(msg_data, msg_type)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            if msg_type == Odometry:
                row = {'time': t, 'vx': msg.twist.twist.linear.x, 'wz': msg.twist.twist.angular.z}
                data[name].append(row)
            elif msg_type == Imu:
                row = {'time': t, 'ax': msg.linear_acceleration.x, 'gx': msg.angular_velocity.x}
                data[name].append(row)

    dfs = {}
    for name, rows in data.items():
        if rows:
            df = pd.DataFrame(rows)
            df['time'] -= df['time'].min()
            dfs[name] = df
            
    return dfs

dfs = load_bag(BAG_PATH)

# Plotting everything
fig, axs = plt.subplots(4, 2, figsize=(20, 24), sharex=True)

# 1. IMU
if 'raw_imu' in dfs and 'rqnn_imu' in dfs:
    axs[0,0].plot(dfs['raw_imu']['time'].to_numpy(), dfs['raw_imu']['ax'].to_numpy(), color='gray', alpha=0.5, label='Raw IMU')
    axs[0,0].plot(dfs['rqnn_imu']['time'].to_numpy(), dfs['rqnn_imu']['ax'].to_numpy(), color='#00ff9d', label='RQNN Filtered')
    axs[0,0].set_title('IMU: Linear Accel (ax)', color='white')
    axs[0,0].legend()

    axs[0,1].plot(dfs['raw_imu']['time'].to_numpy(), dfs['raw_imu']['gx'].to_numpy(), color='gray', alpha=0.5, label='Raw IMU')
    axs[0,1].plot(dfs['rqnn_imu']['time'].to_numpy(), dfs['rqnn_imu']['gx'].to_numpy(), color='#ff007f', label='RQNN Filtered')
    axs[0,1].set_title('IMU: Angular Vel (gx)', color='white')
    axs[0,1].legend()

# 2. Wheel Odom
if 'raw_wheel' in dfs and 'rqnn_wheel' in dfs:
    axs[1,0].plot(dfs['raw_wheel']['time'].to_numpy(), dfs['raw_wheel']['vx'].to_numpy(), color='gray', alpha=0.5, label='Raw Wheel Odom')
    axs[1,0].plot(dfs['rqnn_wheel']['time'].to_numpy(), dfs['rqnn_wheel']['vx'].to_numpy(), color='#00ff9d', label='RQNN Filtered')
    axs[1,0].set_title('Wheel Odom: Linear Vel (vx)', color='white')
    axs[1,0].legend()

    axs[1,1].plot(dfs['raw_wheel']['time'].to_numpy(), dfs['raw_wheel']['wz'].to_numpy(), color='gray', alpha=0.5, label='Raw Wheel Odom')
    axs[1,1].plot(dfs['rqnn_wheel']['time'].to_numpy(), dfs['rqnn_wheel']['wz'].to_numpy(), color='#ff007f', label='RQNN Filtered')
    axs[1,1].set_title('Wheel Odom: Angular Vel (wz)', color='white')
    axs[1,1].legend()

# 3. Visual Odom
if 'raw_visual' in dfs and 'rqnn_visual' in dfs:
    axs[2,0].plot(dfs['raw_visual']['time'].to_numpy(), dfs['raw_visual']['vx'].to_numpy(), color='gray', alpha=0.5, label='Raw Visual Odom')
    axs[2,0].plot(dfs['rqnn_visual']['time'].to_numpy(), dfs['rqnn_visual']['vx'].to_numpy(), color='#00ff9d', label='RQNN Filtered')
    axs[2,0].set_title('Visual Odom: Linear Vel (vx)', color='white')
    axs[2,0].legend()

    axs[2,1].plot(dfs['raw_visual']['time'].to_numpy(), dfs['raw_visual']['wz'].to_numpy(), color='gray', alpha=0.5, label='Raw Visual Odom')
    axs[2,1].plot(dfs['rqnn_visual']['time'].to_numpy(), dfs['rqnn_visual']['wz'].to_numpy(), color='#ff007f', label='RQNN Filtered')
    axs[2,1].set_title('Visual Odom: Angular Vel (wz)', color='white')
    axs[2,1].legend()

# 4. LiDAR Odom
if 'raw_lidar' in dfs and 'rqnn_lidar' in dfs:
    axs[3,0].plot(dfs['raw_lidar']['time'].to_numpy(), dfs['raw_lidar']['vx'].to_numpy(), color='gray', alpha=0.5, label='Raw LiDAR Odom')
    axs[3,0].plot(dfs['rqnn_lidar']['time'].to_numpy(), dfs['rqnn_lidar']['vx'].to_numpy(), color='#00ff9d', label='RQNN Filtered')
    axs[3,0].set_title('LiDAR Odom: Linear Vel (vx)', color='white')
    axs[3,0].legend()

    axs[3,1].plot(dfs['raw_lidar']['time'].to_numpy(), dfs['raw_lidar']['wz'].to_numpy(), color='gray', alpha=0.5, label='Raw LiDAR Odom')
    axs[3,1].plot(dfs['rqnn_lidar']['time'].to_numpy(), dfs['rqnn_lidar']['wz'].to_numpy(), color='#ff007f', label='RQNN Filtered')
    axs[3,1].set_title('LiDAR Odom: Angular Vel (wz)', color='white')
    axs[3,1].legend()

plt.tight_layout()
plt.savefig('/home/anish/.gemini/antigravity/brain/9ef3e9cb-88e5-4c6c-930a-6da9348969d2/all_sensor_comparisons.png', facecolor='black')
print("Successfully generated all_sensor_comparisons.png")
