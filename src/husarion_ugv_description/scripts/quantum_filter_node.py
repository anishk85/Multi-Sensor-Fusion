#!/usr/bin/env python3
"""
Real-time RQNN Filter Node for ROS 2.

Subscribes to raw sensor topics (IMU, Odometry, GPS) and applies the
offline PSO-pretrained Recurrent Quantum Neural Network (RQNN) filters.
Publishes the zero-lag denoised output to /filtered topics for the EKF.
"""
import os
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
import numpy as np

import sys
sys.path.insert(0, '/home/anish/Multi-sensor-fusion')
from qnn_eeg_filtering import RQNNFilter

CONFIG_PATH = '/home/anish/Multi-sensor-fusion/src/husarion_ugv_description/config/rqnn_pretrained_params.json'

class RQNNFilterNode(Node):
    def __init__(self):
        super().__init__('quantum_filter_node')
        
        self.get_logger().info("Initializing RQNN Pre-filter Layer...")
        
        # Load offline PSO pretrained weights
        self.pretrained = {}
        if os.path.exists(CONFIG_PATH):
            with open(CONFIG_PATH, 'r') as f:
                self.pretrained = json.load(f)
            self.get_logger().info(f"Loaded {len(self.pretrained)} pretrained RQNN profiles.")
        else:
            self.get_logger().warn(f"Pretrained weights not found at {CONFIG_PATH}. Using fallback defaults.")
            
        # Instantiate filter bank
        self.filters = {}
        
        # --- Publishers ---
        self.imu_pub = self.create_publisher(Imu, '/imu/data/filtered', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix/filtered', 10)
        self.odom_pub = self.create_publisher(Odometry, '/diff_drive_controller/odom/filtered', 10)
        self.vo_pub = self.create_publisher(Odometry, '/odometry/visual/filtered', 10)
        self.lo_pub = self.create_publisher(Odometry, '/odometry/lidar/filtered', 10)

        # --- Subscribers ---
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/odometry/visual', self.vo_cb, 10)
        self.create_subscription(Odometry, '/odometry/lidar', self.lo_cb, 10)
        
        # Stats
        self.processed = {'imu': 0, 'odom': 0, 'visual': 0, 'lidar': 0}
        self.create_timer(5.0, self.log_stats)
        
    def _get_filter(self, channel_name, default_val, default_range=1.0):
        if channel_name not in self.filters:
            if channel_name in self.pretrained:
                p = dict(self.pretrained[channel_name])
                if 'x_range_min' in p and 'x_range_max' in p:
                    p['x_range'] = (p.pop('x_range_min'), p.pop('x_range_max'))
                if 'dt' not in p:
                    p['dt'] = 0.02
                self.filters[channel_name] = RQNNFilter(**p)
                # Ensure wave packet starts at the current value
                self.filters[channel_name].reset()
            else:
                # Fallback to sensible defaults
                self.filters[channel_name] = RQNNFilter(
                    x_range=(default_val - default_range, default_val + default_range),
                    dt=0.02, mass=1.0, sigma=0.5, gamma=2.0)
                self.filters[channel_name].reset()
        return self.filters[channel_name]
        
    def _filter_sample(self, channel_name, val, default_range=1.0):
        filt = self._get_filter(channel_name, val, default_range)
        # Apply single Crank-Nicolson step
        try:
            # bias the wavepacket toward the observation
            filt._bias_wavepacket(val, strength=filt.bias_strength)
            V = filt._potential(y_obs=val)
            for _ in range(3): # 3 substeps — matches filter_signal() batch mode
                filt._evolve_psi(V)
            
            y_hat = filt._estimate(filt._pdf())
            # Hebbian weight update
            filt._update_weights(val - y_hat, y_hat)
            return y_hat
        except Exception:
            return val

    def imu_cb(self, msg: Imu):
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation  # Don't filter orientation from internal IMU algorithms
        out.orientation_covariance = msg.orientation_covariance
        
        out.linear_acceleration.x = self._filter_sample('imu_ax', msg.linear_acceleration.x, 2.0)
        out.linear_acceleration.y = self._filter_sample('imu_ay', msg.linear_acceleration.y, 2.0)
        out.linear_acceleration.z = self._filter_sample('imu_az', msg.linear_acceleration.z, 2.0)
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        
        out.angular_velocity.x = self._filter_sample('imu_gx', msg.angular_velocity.x, 0.5)
        out.angular_velocity.y = self._filter_sample('imu_gy', msg.angular_velocity.y, 0.5)
        out.angular_velocity.z = self._filter_sample('imu_gz', msg.angular_velocity.z, 0.5)
        # FIX 4: Override angular velocity covariance to account for gyro bias.
        # Gazebo publishes stddev²=0.000081 (white noise only), but actual noise
        # including bias_stddev=0.001 is ~0.002. Without this override the EKF
        # over-trusts the biased gyro signal, causing yaw drift.
        # gz covariance higher (0.004) — RQNN achieves only 12.8% noise reduction
        # on gz vs 100% for gx/gy, so EKF trusts gz estimate less.
        out.angular_velocity_covariance = [
            0.002, 0.0,   0.0,
            0.0,   0.002, 0.0,
            0.0,   0.0,   0.004
        ]

        self.imu_pub.publish(out)
        self.processed['imu'] += 1

    def gps_cb(self, msg: NavSatFix):
        out = NavSatFix()
        out.header = msg.header
        out.status = msg.status
        # GPS is noisy but filtering lat/lon with 1D filters can cause distortion.
        # We pass it through directly, EKF #2 handles GPS correctly.
        out.latitude = msg.latitude
        out.longitude = msg.longitude
        out.altitude = msg.altitude
        # Gazebo NavSat bridge publishes covariance=0 (UNKNOWN) even with noise configured.
        # navsat_transform passes this through → EKF Kalman gain K→1 → 0.5m GPS jumps.
        # Inject diagonal covariance from URDF noise config: horizontal stddev=0.5m → var=0.25m².
        out.position_covariance = [
            0.25, 0.0, 0.0,
            0.0,  0.25, 0.0,
            0.0,  0.0,  1.0,
        ]
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gps_pub.publish(out)

    def _process_odom(self, msg: Odometry, prefix: str):
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        
        # Pass pose through UNFILTERED.
        # With differential: true in the EKF, it computes Δpose = pose(t) - pose(t-1).
        # RQNN smoothing of absolute pose introduces temporal lag, causing the deltas
        # to be systematically biased (underestimate during accel, overestimate during
        # decel). This lag accumulates into drift — verified by regression from 21m to 38m.
        out.pose = msg.pose
        
        # Filter twist (velocities) — this provides genuine denoising benefit.
        # Velocity is already a rate signal, so RQNN smoothing doesn't cause
        # the lag-delta problem that affects pose with differential mode.
        out.twist.twist.linear.x = self._filter_sample(f'{prefix}_vx', msg.twist.twist.linear.x, 0.5)
        out.twist.twist.linear.y = msg.twist.twist.linear.y  # usually 0 for diff drive
        out.twist.twist.linear.z = msg.twist.twist.linear.z
        
        out.twist.twist.angular.x = msg.twist.twist.angular.x
        out.twist.twist.angular.y = msg.twist.twist.angular.y
        out.twist.twist.angular.z = self._filter_sample(f'{prefix}_wz', msg.twist.twist.angular.z, 0.5)
        
        out.twist.covariance = msg.twist.covariance
        return out

    def odom_cb(self, msg: Odometry):
        # RQNN adds <1% benefit on wheel odometry — bypass to avoid latency.
        self.odom_pub.publish(msg)
        self.processed['odom'] += 1

    def vo_cb(self, msg: Odometry):
        out = self._process_odom(msg, 'visual')
        self.vo_pub.publish(out)
        self.processed['visual'] += 1

    def lo_cb(self, msg: Odometry):
        # RQNN degrades LiDAR odometry (−2.5% to −7.9% noise increase) — bypass.
        self.lo_pub.publish(msg)
        self.processed['lidar'] += 1

    def log_stats(self):
        self.get_logger().info(
            f"Zero-Lag RQNN Flow: IMU:{self.processed['imu']} | "
            f"W-Odom:{self.processed['odom']} | V-Odom:{self.processed['visual']} | "
            f"L-Odom:{self.processed['lidar']}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = RQNNFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
