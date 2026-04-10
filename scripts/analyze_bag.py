import os
import sys
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader
from rosbags.typesys import RecordsByType

def process_bag(bag_path):
    print(f"Reading bag: {bag_path}")
    
    imu_data = []
    gps_data = []
    
    with AnyReader([Path(bag_path)]) as reader:
        # Check available topics
        print("Topics found:", list(reader.topics.keys()))
        
        for connection, timestamp, rawdata in reader.messages():
            msg = reader.deserialize(rawdata, connection.msgtype)
            
            if connection.topic == '/imu/data':
                imu_data.append({
                    'timestamp': timestamp,
                    'accel_x': msg.linear_acceleration.x,
                    'accel_y': msg.linear_acceleration.y,
                    'accel_z': msg.linear_acceleration.z,
                    'gyro_x': msg.angular_velocity.x,
                    'gyro_y': msg.angular_velocity.y,
                    'gyro_z': msg.angular_velocity.z,
                })
            
            elif connection.topic == '/gps/fix':
                gps_data.append({
                    'timestamp': timestamp,
                    'lat': msg.latitude,
                    'lon': msg.longitude,
                    'alt': msg.altitude,
                })

    return pd.DataFrame(imu_data), pd.DataFrame(gps_data)

def analyze_and_plot(df, name, expected_std=None):
    if df.empty:
        print(f"No data found for {name}")
        return

    print(f"\n--- {name} Noise Analysis ---")
    numeric_cols = [c for c in df.columns if c != 'timestamp']
    stats = df[numeric_cols].agg(['mean', 'std', 'var']).T
    stats.columns = ['Mean (Bias)', 'StdDev (Measured)', 'Variance']
    
    if expected_std:
        stats['Expected (URDF)'] = expected_std
        stats['Error %'] = np.abs((stats['StdDev (Measured)'] - expected_std) / expected_std) * 100
    
    print(stats.to_string())

    # Plotting
    fig, axes = plt.subplots(len(numeric_cols), 1, figsize=(10, 2*len(numeric_cols)))
    if len(numeric_cols) == 1: axes = [axes]
    
    for i, col in enumerate(numeric_cols):
        axes[i].plot(df['timestamp'] - df['timestamp'].iloc[0], df[col], label='Measured')
        axes[i].set_title(f"{name}: {col}")
        axes[i].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    bag_dir = "sensor_burst"
    if not os.path.exists(bag_dir):
        print(f"Error: Folder '{bag_dir}' not found. Record data first using:")
        print("ros2 bag record -o sensor_burst /imu/data /gps/fix")
        sys.exit(1)

    imu_df, gps_df = process_bag(bag_dir)
    
    # Values from your URDF: accel std=0.021, gyro std=0.009
    analyze_and_plot(imu_df, "IMU", expected_std=0.021) 
    analyze_and_plot(gps_df, "GPS", expected_std=0.5)
