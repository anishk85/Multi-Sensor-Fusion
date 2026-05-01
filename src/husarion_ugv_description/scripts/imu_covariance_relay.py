#!/usr/bin/env python3
"""
IMU Covariance Relay Node

Gazebo publishes IMU angular_velocity_covariance = stddev² = 0.009² = 0.000081
(white noise only). EKF trusts this 25× more than it should because the actual
noise includes bias_stddev=0.001 rad/s from the URDF noise model.

Over-trusted IMU gyro (covariance 0.000081) vs LiDAR yaw (covariance 0.02):
  IMU information density: 200Hz / 0.000081 ≈ 2.5M/s
  LiDAR information density:  5Hz / 0.02     ≈ 250/s
  Ratio: ~10,000× → LiDAR yaw correction is effectively zero → map frame rotates.

This relay republishes /imu/data with corrected angular_velocity_covariance
so the EKF properly weights IMU vs LiDAR for yaw. Values match the quantum
filter node's override (quantum_filter_node.py:115-119).

  gx/gy: 0.002  (white 0.000081 + bias contribution, rounded)
  gz:    0.004  (higher: gz bias less stable, orientation drift dominates yaw)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

GYRO_COV_XY = 0.002
GYRO_COV_Z  = 0.004


class ImuCovarianceRelay(Node):
    def __init__(self):
        super().__init__('imu_covariance_relay')
        self.pub = self.create_publisher(Imu, '/imu/data/covariance_fixed', 10)
        self.create_subscription(Imu, '/imu/data', self.cb, 10)

    def cb(self, msg: Imu):
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = [
            GYRO_COV_XY, 0.0, 0.0,
            0.0, GYRO_COV_XY, 0.0,
            0.0, 0.0, GYRO_COV_Z,
        ]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuCovarianceRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
