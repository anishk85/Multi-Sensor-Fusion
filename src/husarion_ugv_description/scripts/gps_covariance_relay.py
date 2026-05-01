#!/usr/bin/env python3
"""
GPS Covariance Relay Node

Gazebo NavSat bridge publishes NavSatFix with position_covariance=0 (covariance_type=UNKNOWN)
even when noise is configured in the URDF. robot_localization navsat_transform passes this
through to /odometry/gps, causing the EKF to treat GPS as a perfect measurement (K→1)
and slam to every noisy reading.

This node republishes /gps/fix with realistic covariance values derived from the URDF
noise config: horizontal stddev = 4.5e-6 deg ≈ 0.5 m → variance = 0.25 m².
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

GPS_VAR_HORIZONTAL = 9.0    # 3.0 m stddev → 9.0 m² variance (matches observed Gazebo GPS scatter σ≈0.5m range≈2.4m)
GPS_VAR_VERTICAL   = 99.0   # 10.0 m stddev — not used in 2D mode


class GPSCovarianceRelay(Node):
    def __init__(self):
        super().__init__('gps_covariance_relay')
        self.pub = self.create_publisher(NavSatFix, '/gps/fix/covariance_fixed', 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.cb, 10)

    def cb(self, msg: NavSatFix):
        out = NavSatFix()
        out.header = msg.header
        out.status = msg.status
        out.latitude = msg.latitude
        out.longitude = msg.longitude
        out.altitude = msg.altitude
        # Inject diagonal covariance: [xx, xy, xz, yx, yy, yz, zx, zy, zz]
        out.position_covariance = [
            GPS_VAR_HORIZONTAL, 0.0, 0.0,
            0.0, GPS_VAR_HORIZONTAL, 0.0,
            0.0, 0.0, GPS_VAR_VERTICAL,
        ]
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(out)


def main():
    rclpy.init()
    node = GPSCovarianceRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()