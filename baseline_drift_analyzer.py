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
plt.rcParams['figure.figsize'] = (14, 8)
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
        '/odometry/local': ('local_ekf', Odometry),
        '/odometry/global': ('global_ekf', Odometry),
    }

    data = {k: [] for k, _ in TOPICS.values()}

    while reader.has_next():
        topic, msg_data, timestamp = reader.read_next()
        if topic in TOPICS:
            name, msg_type = TOPICS[topic]
            msg = deserialize_message(msg_data, msg_type)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            row = {'time': t, 'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y}
            data[name].append(row)

    dfs = {}
    for name, rows in data.items():
        if rows:
            df = pd.DataFrame(rows)
            df['time'] -= df['time'].min()
            dfs[name] = df
            
    return dfs

dfs = load_bag(BAG_PATH)

fig, (ax_map, ax_drift) = plt.subplots(1, 2, figsize=(20, 9))
if 'local_ekf' in dfs and 'global_ekf' in dfs:
    loc = dfs['local_ekf']
    glob = dfs['global_ekf']
    
    ax_map.plot(loc['x'].to_numpy(), loc['y'].to_numpy(), label='Baseline Local EKF (NO RQNN)', color='#ffaa00', linewidth=2.5)
    ax_map.plot(glob['x'].to_numpy(), glob['y'].to_numpy(), label='Baseline Global EKF (GPS Corr)', color='#00aaff', linewidth=2.5, linestyle='--')
    ax_map.scatter([loc['x'].iloc[0]], [loc['y'].iloc[0]], color='green', s=150, zorder=5, label='Start')
    ax_map.scatter([loc['x'].iloc[-1]], [loc['y'].iloc[-1]], color='red', s=150, zorder=5, label='End Local')
    ax_map.set_title('Baseline Unfiltered Top-Down Trajectory', fontsize=16, color='white')
    ax_map.legend()
    ax_map.axis('equal')
    
    t_min = max(loc['time'].min(), glob['time'].min())
    t_max = min(loc['time'].max(), glob['time'].max())
    times = np.linspace(t_min, t_max, 500)
    
    loc_x_interp = np.interp(times, loc['time'].to_numpy(), loc['x'].to_numpy())
    loc_y_interp = np.interp(times, loc['time'].to_numpy(), loc['y'].to_numpy())
    glob_x_interp = np.interp(times, glob['time'].to_numpy(), glob['x'].to_numpy())
    glob_y_interp = np.interp(times, glob['time'].to_numpy(), glob['y'].to_numpy())
    
    instant_drift = np.sqrt((loc_x_interp - glob_x_interp)**2 + (loc_y_interp - glob_y_interp)**2)
    
    ax_drift.plot(times, instant_drift, color='#ff003c', linewidth=3)
    ax_drift.fill_between(times, instant_drift, color='#ff003c', alpha=0.2)
    ax_drift.set_title(f'Baseline Unfiltered Positional Drift\\nFinal Max Drift: {instant_drift[-1]:.3f} meters', fontsize=16, color='white')
    ax_drift.set_xlabel('Time (s)', fontsize=14, color='white')
    ax_drift.set_ylabel('Drift Error (meters)', fontsize=14, color='white')
    
plt.tight_layout()
plt.savefig('/home/anish/.gemini/antigravity/brain/9ef3e9cb-88e5-4c6c-930a-6da9348969d2/baseline_drift_overview.png', facecolor='black')
print("Successfully generated baseline_drift_overview.png")
