#!/usr/bin/env python3
"""
Post-EKF Fusion Analysis Script for Husarion Lynx
==================================================

Records a rosbag with raw + fused topics, then plots comparisons.

Usage:
  1. Launch the full system:
       ros2 launch husarion_ugv_description dual_ekf_navsat.launch.py

  2. Drive the robot (teleop or Nav2):
       ros2 run teleop_twist_keyboard teleop_twist_keyboard

  3. Record data for ~60 seconds:
       ros2 bag record -o ekf_fusion_run \
         /imu/data \
         /gps/fix \
         /diff_drive_controller/odom \
         /odometry/local \
         /odometry/global \
         /odometry/gps \
         /cmd_vel

  4. Run this script:
       python3 scripts/analyze_ekf_fusion.py ekf_fusion_run

  The script will produce 5 comparison plots:
    1. XY trajectory: raw odom vs EKF local vs EKF global vs GPS
    2. Velocity: commanded vs raw odom vs EKF-fused
    3. Yaw: raw odom vs IMU-fused EKF estimate
    4. GPS scatter vs EKF position (noise reduction check)
    5. Innovation analysis (GPS position minus EKF estimate)
"""

import os
import sys
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

# Use rosbags (pure Python, no ROS needed to run this script)
from rosbags.highlevel import AnyReader

# ── Styling ──────────────────────────────────────────────────────────────────
plt.rcParams.update({
    'figure.facecolor': '#1a1a2e',
    'axes.facecolor': '#16213e',
    'axes.edgecolor': '#e0e0e0',
    'axes.labelcolor': '#e0e0e0',
    'text.color': '#e0e0e0',
    'xtick.color': '#aaaaaa',
    'ytick.color': '#aaaaaa',
    'grid.color': '#2a2a4a',
    'grid.alpha': 0.5,
    'legend.facecolor': '#16213e',
    'legend.edgecolor': '#444444',
    'font.size': 11,
})

COLORS = {
    'raw_odom':   '#ff6b6b',   # red
    'ekf_local':  '#51cf66',   # green
    'ekf_global': '#339af0',   # blue
    'gps':        '#ffd43b',   # yellow
    'gps_odom':   '#ff922b',   # orange
    'cmd_vel':    '#cc5de8',   # purple
    'imu':        '#20c997',   # teal
}


def extract_topics(bag_path):
    """Read a rosbag and extract all relevant topics into DataFrames."""

    data = {
        'raw_odom': [],     # /diff_drive_controller/odom
        'ekf_local': [],    # /odometry/local  (EKF #1)
        'ekf_global': [],   # /odometry/global (EKF #2)
        'gps_fix': [],      # /gps/fix
        'gps_odom': [],     # /odometry/gps (navsat output)
        'cmd_vel': [],      # /cmd_vel
        'imu': [],          # /imu/data
    }

    with AnyReader([Path(bag_path)]) as reader:
        print(f"Reading bag: {bag_path}")
        print(f"Topics found: {list(reader.topics.keys())}")

        for connection, timestamp, rawdata in reader.messages():
            msg = reader.deserialize(rawdata, connection.msgtype)
            t_sec = timestamp * 1e-9  # nanoseconds to seconds

            if connection.topic == '/diff_drive_controller/odom':
                data['raw_odom'].append({
                    't': t_sec,
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'vx': msg.twist.twist.linear.x,
                    'vyaw': msg.twist.twist.angular.z,
                    # Extract yaw from quaternion
                    'yaw': _quat_to_yaw(msg.pose.pose.orientation),
                })

            elif connection.topic == '/odometry/local':
                data['ekf_local'].append({
                    't': t_sec,
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'vx': msg.twist.twist.linear.x,
                    'vyaw': msg.twist.twist.angular.z,
                    'yaw': _quat_to_yaw(msg.pose.pose.orientation),
                })

            elif connection.topic == '/odometry/global':
                data['ekf_global'].append({
                    't': t_sec,
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'vx': msg.twist.twist.linear.x,
                    'vyaw': msg.twist.twist.angular.z,
                    'yaw': _quat_to_yaw(msg.pose.pose.orientation),
                })

            elif connection.topic == '/gps/fix':
                data['gps_fix'].append({
                    't': t_sec,
                    'lat': msg.latitude,
                    'lon': msg.longitude,
                    'alt': msg.altitude,
                })

            elif connection.topic == '/odometry/gps':
                data['gps_odom'].append({
                    't': t_sec,
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                })

            elif connection.topic == '/cmd_vel':
                data['cmd_vel'].append({
                    't': t_sec,
                    'vx': msg.linear.x,
                    'vyaw': msg.angular.z,
                })

            elif connection.topic == '/imu/data':
                data['imu'].append({
                    't': t_sec,
                    'yaw': _quat_to_yaw(msg.orientation),
                    'gyro_z': msg.angular_velocity.z,
                    'accel_x': msg.linear_acceleration.x,
                    'accel_y': msg.linear_acceleration.y,
                })

    # Convert to DataFrames, normalize time to start at 0
    dfs = {}
    t0 = float('inf')
    for key, rows in data.items():
        if rows:
            df = pd.DataFrame(rows)
            t0 = min(t0, df['t'].iloc[0])
            dfs[key] = df

    for key in dfs:
        dfs[key]['t'] -= t0

    return dfs


def _quat_to_yaw(q):
    """Extract yaw from a geometry_msgs/Quaternion using atan2."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)


# ── Plot 1: XY Trajectory Comparison ────────────────────────────────────────
def plot_xy_trajectory(dfs):
    """Compare XY paths: raw odom vs EKF local vs EKF global vs GPS."""
    fig, ax = plt.subplots(figsize=(10, 8))
    fig.suptitle("XY Trajectory Comparison", fontsize=16, fontweight='bold')

    if 'raw_odom' in dfs:
        ax.plot(dfs['raw_odom']['x'], dfs['raw_odom']['y'],
                color=COLORS['raw_odom'], alpha=0.6, linewidth=1.5,
                label='Raw Wheel Odom')

    if 'ekf_local' in dfs:
        ax.plot(dfs['ekf_local']['x'], dfs['ekf_local']['y'],
                color=COLORS['ekf_local'], linewidth=2,
                label='EKF Local (odom frame)')

    if 'ekf_global' in dfs:
        ax.plot(dfs['ekf_global']['x'], dfs['ekf_global']['y'],
                color=COLORS['ekf_global'], linewidth=2,
                label='EKF Global (map frame)')

    if 'gps_odom' in dfs:
        ax.scatter(dfs['gps_odom']['x'], dfs['gps_odom']['y'],
                   color=COLORS['gps'], s=15, alpha=0.5, zorder=5,
                   label='GPS (navsat → local XY)')

    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.legend(loc='upper left')
    ax.set_aspect('equal')
    ax.grid(True)
    plt.tight_layout()
    plt.savefig("plots/ekf_xy_trajectory.png", dpi=150, bbox_inches='tight')
    print("Saved: plots/ekf_xy_trajectory.png")
    plt.show()


# ── Plot 2: Velocity Comparison ─────────────────────────────────────────────
def plot_velocity(dfs):
    """Compare linear and angular velocity: cmd_vel vs odom vs EKF."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("Velocity: Commanded vs Raw vs EKF-Fused", fontsize=16,
                 fontweight='bold')

    # Linear velocity
    if 'cmd_vel' in dfs:
        ax1.plot(dfs['cmd_vel']['t'], dfs['cmd_vel']['vx'],
                 color=COLORS['cmd_vel'], alpha=0.7, linewidth=1,
                 label='cmd_vel (commanded)')
    if 'raw_odom' in dfs:
        ax1.plot(dfs['raw_odom']['t'], dfs['raw_odom']['vx'],
                 color=COLORS['raw_odom'], alpha=0.5, linewidth=1,
                 label='Raw odom vx')
    if 'ekf_local' in dfs:
        ax1.plot(dfs['ekf_local']['t'], dfs['ekf_local']['vx'],
                 color=COLORS['ekf_local'], linewidth=2,
                 label='EKF Local vx')

    ax1.set_ylabel("Linear Velocity (m/s)")
    ax1.legend(loc='upper right')
    ax1.grid(True)

    # Angular velocity
    if 'cmd_vel' in dfs:
        ax2.plot(dfs['cmd_vel']['t'], dfs['cmd_vel']['vyaw'],
                 color=COLORS['cmd_vel'], alpha=0.7, linewidth=1,
                 label='cmd_vel (commanded)')
    if 'raw_odom' in dfs:
        ax2.plot(dfs['raw_odom']['t'], dfs['raw_odom']['vyaw'],
                 color=COLORS['raw_odom'], alpha=0.5, linewidth=1,
                 label='Raw odom vyaw')
    if 'ekf_local' in dfs:
        ax2.plot(dfs['ekf_local']['t'], dfs['ekf_local']['vyaw'],
                 color=COLORS['ekf_local'], linewidth=2,
                 label='EKF Local vyaw')

    ax2.set_xlabel("Time (seconds)")
    ax2.set_ylabel("Angular Velocity (rad/s)")
    ax2.legend(loc='upper right')
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig("plots/ekf_velocity_comparison.png", dpi=150, bbox_inches='tight')
    print("Saved: plots/ekf_velocity_comparison.png")
    plt.show()


# ── Plot 3: Yaw Comparison ──────────────────────────────────────────────────
def plot_yaw(dfs):
    """Compare yaw estimates: raw odom vs EKF (IMU-fused)."""
    fig, ax = plt.subplots(figsize=(14, 5))
    fig.suptitle("Yaw Estimate: Raw Odom vs EKF (IMU-Fused)", fontsize=16,
                 fontweight='bold')

    if 'raw_odom' in dfs:
        ax.plot(dfs['raw_odom']['t'], np.degrees(dfs['raw_odom']['yaw']),
                color=COLORS['raw_odom'], alpha=0.5, linewidth=1,
                label='Raw odom yaw')
    if 'ekf_local' in dfs:
        ax.plot(dfs['ekf_local']['t'], np.degrees(dfs['ekf_local']['yaw']),
                color=COLORS['ekf_local'], linewidth=2,
                label='EKF Local yaw (IMU-fused)')
    if 'ekf_global' in dfs:
        ax.plot(dfs['ekf_global']['t'], np.degrees(dfs['ekf_global']['yaw']),
                color=COLORS['ekf_global'], linewidth=2, linestyle='--',
                label='EKF Global yaw')

    ax.set_xlabel("Time (seconds)")
    ax.set_ylabel("Yaw (degrees)")
    ax.legend(loc='upper right')
    ax.grid(True)

    plt.tight_layout()
    plt.savefig("plots/ekf_yaw_comparison.png", dpi=150, bbox_inches='tight')
    print("Saved: plots/ekf_yaw_comparison.png")
    plt.show()


# ── Plot 4: GPS Noise Reduction ──────────────────────────────────────────────
def plot_gps_noise_reduction(dfs):
    """Show how the EKF smooths out GPS noise."""
    if 'gps_odom' not in dfs or 'ekf_global' not in dfs:
        print("Skipping GPS noise reduction plot — missing data")
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("GPS Noise Reduction by EKF", fontsize=16, fontweight='bold')

    gps = dfs['gps_odom']
    ekf = dfs['ekf_global']

    # Left: X over time
    ax1.scatter(gps['t'], gps['x'], color=COLORS['gps'], s=8, alpha=0.4,
                label='GPS x (noisy)')
    ax1.plot(ekf['t'], ekf['x'], color=COLORS['ekf_global'], linewidth=2,
             label='EKF Global x (fused)')
    ax1.set_xlabel("Time (seconds)")
    ax1.set_ylabel("X position (meters)")
    ax1.legend()
    ax1.grid(True)

    # Right: Y over time
    ax2.scatter(gps['t'], gps['y'], color=COLORS['gps'], s=8, alpha=0.4,
                label='GPS y (noisy)')
    ax2.plot(ekf['t'], ekf['y'], color=COLORS['ekf_global'], linewidth=2,
             label='EKF Global y (fused)')
    ax2.set_xlabel("Time (seconds)")
    ax2.set_ylabel("Y position (meters)")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig("plots/ekf_gps_noise_reduction.png", dpi=150, bbox_inches='tight')
    print("Saved: plots/ekf_gps_noise_reduction.png")
    plt.show()


# ── Plot 5: Innovation (GPS - EKF) ──────────────────────────────────────────
def plot_innovation(dfs):
    """
    Plot the 'innovation' — difference between GPS position and EKF estimate.
    If the EKF is well-tuned, this should be zero-mean Gaussian noise.
    """
    if 'gps_odom' not in dfs or 'ekf_global' not in dfs:
        print("Skipping innovation plot — missing data")
        return

    gps = dfs['gps_odom']
    ekf = dfs['ekf_global']

    # Interpolate EKF position at GPS timestamps
    ekf_x_interp = np.interp(gps['t'], ekf['t'], ekf['x'])
    ekf_y_interp = np.interp(gps['t'], ekf['t'], ekf['y'])

    innov_x = gps['x'].values - ekf_x_interp
    innov_y = gps['y'].values - ekf_y_interp

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle("Innovation Analysis (GPS − EKF Estimate)", fontsize=16,
                 fontweight='bold')

    # Innovation over time
    axes[0].plot(gps['t'], innov_x, color=COLORS['gps'], alpha=0.6, label='Δx')
    axes[0].plot(gps['t'], innov_y, color=COLORS['ekf_global'], alpha=0.6,
                 label='Δy')
    axes[0].axhline(0, color='white', linewidth=0.5, linestyle='--')
    axes[0].set_xlabel("Time (seconds)")
    axes[0].set_ylabel("Innovation (meters)")
    axes[0].legend()
    axes[0].grid(True)
    axes[0].set_title("Time Series")

    # Histogram
    axes[1].hist(innov_x, bins=30, color=COLORS['gps'], alpha=0.6, label='Δx',
                 density=True)
    axes[1].hist(innov_y, bins=30, color=COLORS['ekf_global'], alpha=0.6,
                 label='Δy', density=True)
    axes[1].set_xlabel("Innovation (meters)")
    axes[1].set_ylabel("Density")
    axes[1].legend()
    axes[1].grid(True)
    axes[1].set_title("Histogram (should be Gaussian)")

    # Scatter (should be circular cluster around origin)
    axes[2].scatter(innov_x, innov_y, color=COLORS['gps'], s=10, alpha=0.4)
    axes[2].axhline(0, color='white', linewidth=0.5, linestyle='--')
    axes[2].axvline(0, color='white', linewidth=0.5, linestyle='--')
    axes[2].set_xlabel("Δx (meters)")
    axes[2].set_ylabel("Δy (meters)")
    axes[2].set_aspect('equal')
    axes[2].grid(True)
    axes[2].set_title("Innovation Scatter")

    # Print statistics
    print(f"\n--- Innovation Statistics ---")
    print(f"  Δx: mean={innov_x.mean():.4f} m, std={innov_x.std():.4f} m")
    print(f"  Δy: mean={innov_y.mean():.4f} m, std={innov_y.std():.4f} m")
    print(f"  RMSE: {np.sqrt(np.mean(innov_x**2 + innov_y**2)):.4f} m")

    plt.tight_layout()
    plt.savefig("plots/ekf_innovation.png", dpi=150, bbox_inches='tight')
    print("Saved: plots/ekf_innovation.png")
    plt.show()


# ── Summary Statistics ───────────────────────────────────────────────────────
def print_summary(dfs):
    """Print a summary table of all data sources."""
    print("\n" + "=" * 70)
    print("  POST-FUSION DATA SUMMARY")
    print("=" * 70)

    for name, df in dfs.items():
        duration = df['t'].iloc[-1] - df['t'].iloc[0] if len(df) > 1 else 0
        rate = len(df) / duration if duration > 0 else 0
        print(f"  {name:20s}  {len(df):6d} msgs  {duration:7.1f}s  ~{rate:5.1f} Hz")

    print("=" * 70)

    # EKF drift analysis (how much EKF local drifted vs EKF global)
    if 'ekf_local' in dfs and 'ekf_global' in dfs:
        local = dfs['ekf_local']
        glob = dfs['ekf_global']
        # Compare final positions
        dx = local['x'].iloc[-1] - glob['x'].iloc[-1]
        dy = local['y'].iloc[-1] - glob['y'].iloc[-1]
        drift = np.sqrt(dx**2 + dy**2)
        duration = local['t'].iloc[-1]
        print(f"\n  EKF Drift (local vs global at end): {drift:.3f} m over {duration:.1f}s")
        print(f"  Drift rate: {drift / duration * 60:.3f} m/min")


# ── Main ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 scripts/analyze_ekf_fusion.py <bag_directory>")
        print()
        print("First, record data while running the dual EKF system:")
        print("  ros2 bag record -o ekf_fusion_run \\")
        print("    /imu/data /gps/fix /diff_drive_controller/odom \\")
        print("    /odometry/local /odometry/global /odometry/gps /cmd_vel")
        sys.exit(1)

    bag_dir = sys.argv[1]
    if not os.path.exists(bag_dir):
        print(f"Error: Bag directory '{bag_dir}' not found.")
        sys.exit(1)

    # Ensure output directory exists
    os.makedirs("plots", exist_ok=True)

    # Extract all data
    dfs = extract_topics(bag_dir)
    print_summary(dfs)

    # Generate all plots
    plot_xy_trajectory(dfs)
    plot_velocity(dfs)
    plot_yaw(dfs)
    plot_gps_noise_reduction(dfs)
    plot_innovation(dfs)

    print("\n✅ All plots saved to plots/ directory.")
    print("   Run `ls plots/ekf_*` to see all generated files.")
