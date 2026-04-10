#!/usr/bin/env python3
"""
Quantum-Inspired Wavelet Denoising Filter Node for ROS 2.

This node subscribes to raw sensor topics (IMU, GPS, Odom, Visual Odom,
LiDAR Odom), applies Quantum Particle Swarm Optimization (QPSO) optimized
wavelet denoising to each signal channel, and publishes the cleaned data
on /filtered topics.

Architecture:
  Raw Sensors → [This Node] → Filtered Sensors → [EKF] → Fused Pose

Algorithm:
  1. Buffer incoming samples in a sliding window
  2. Apply Discrete Wavelet Transform (DWT) using db4 wavelet
  3. Use QPSO to find optimal soft-thresholding parameter
  4. Apply soft-thresholding to detail coefficients
  5. Reconstruct via Inverse DWT
  6. Publish the latest denoised sample
"""

import time
import numpy as np

try:
    import pywt
except ImportError:
    raise ImportError(
        "PyWavelets is required: pip install PyWavelets"
    )

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import copy


# ═══════════════════════════════════════════════════════════════════════════
#  QPSO Wavelet Denoiser — the "Quantum" part
# ═══════════════════════════════════════════════════════════════════════════

class QPSOWaveletDenoiser:
    """
    Quantum Particle Swarm Optimization (QPSO) based wavelet denoiser.

    Uses QPSO to find the optimal soft-thresholding parameter for
    wavelet coefficient denoising.
    """

    def __init__(self, wavelet='db4', level=3, n_particles=10,
                 max_iter=8, alpha=0.7):
        """
        Args:
            wavelet:      Wavelet family (default: Daubechies-4)
            level:        DWT decomposition levels
            n_particles:  QPSO swarm size
            max_iter:     QPSO iterations per denoise call
            alpha:        Fitness weight (α*fidelity + (1-α)*smoothness)
        """
        self.wavelet = wavelet
        self.level = level
        self.n_particles = n_particles
        self.max_iter = max_iter
        self.alpha = alpha

    def _estimate_noise_sigma(self, detail_coeffs):
        """Robust noise estimation from finest-level detail coefficients (MAD)."""
        return np.median(np.abs(detail_coeffs)) / 0.6745

    def _soft_threshold(self, coeffs, lam):
        """Apply soft thresholding to wavelet coefficients."""
        return np.sign(coeffs) * np.maximum(np.abs(coeffs) - lam, 0.0)

    def _reconstruct(self, cA, detail_list, thresholds):
        """Reconstruct signal from thresholded wavelet coefficients."""
        thresholded = [self._soft_threshold(d, t)
                       for d, t in zip(detail_list, thresholds)]
        coeffs = [cA] + thresholded
        return pywt.waverec(coeffs, self.wavelet)

    def _fitness(self, signal, thresholds, cA, detail_list):
        """
        Fitness = α * reconstruction_error + (1-α) * roughness

        Lower is better.
        """
        reconstructed = self._reconstruct(cA, detail_list, thresholds)
        # Truncate to original length (waverec may pad)
        reconstructed = reconstructed[:len(signal)]

        # Fidelity: how close is denoised to original
        fidelity = np.mean((signal - reconstructed) ** 2)

        # Smoothness: penalize remaining high-frequency content
        roughness = np.mean(np.diff(reconstructed) ** 2)

        return self.alpha * fidelity + (1.0 - self.alpha) * roughness

    def denoise(self, signal):
        """
        Denoise a 1-D signal using QPSO-optimized wavelet thresholding.

        Args:
            signal: 1-D numpy array of noisy samples

        Returns:
            1-D numpy array of denoised samples (same length)
        """
        if len(signal) < 2 ** self.level:
            return signal  # Too short for meaningful decomposition

        # --- DWT Decomposition ---
        coeffs = pywt.wavedec(signal, self.wavelet, level=self.level)
        cA = coeffs[0]
        detail_list = coeffs[1:]  # [cD_L, cD_{L-1}, ..., cD_1]

        # Noise estimate from finest detail
        sigma = self._estimate_noise_sigma(detail_list[-1])
        n = len(signal)

        # Universal threshold as upper bound
        lambda_max = sigma * np.sqrt(2.0 * np.log(n))
        n_levels = len(detail_list)

        # --- QPSO Optimization ---
        # Each particle is a vector of thresholds, one per decomposition level
        particles = np.random.uniform(
            0, lambda_max, (self.n_particles, n_levels)
        )
        pbest = particles.copy()
        pbest_fitness = np.full(self.n_particles, np.inf)
        gbest = particles[0].copy()
        gbest_fitness = np.inf

        # Evaluate initial population
        for i in range(self.n_particles):
            f = self._fitness(signal, particles[i], cA, detail_list)
            pbest_fitness[i] = f
            if f < gbest_fitness:
                gbest_fitness = f
                gbest = particles[i].copy()

        # QPSO iterations
        for iteration in range(self.max_iter):
            # Contraction-expansion coefficient (decays from 1.0 to 0.5)
            beta = 1.0 - 0.5 * iteration / max(self.max_iter - 1, 1)

            # Mean best position
            mbest = np.mean(pbest, axis=0)

            for i in range(self.n_particles):
                phi = np.random.uniform(0, 1, n_levels)
                u = np.random.uniform(1e-10, 1, n_levels)

                # Attractor
                p = phi * pbest[i] + (1 - phi) * gbest

                # Quantum-inspired position update
                sign = np.where(np.random.random(n_levels) < 0.5, -1.0, 1.0)
                particles[i] = p + sign * beta * np.abs(mbest - particles[i]) * np.log(1.0 / u)

                # Clamp to valid range
                particles[i] = np.clip(particles[i], 0, lambda_max)

                # Evaluate
                f = self._fitness(signal, particles[i], cA, detail_list)
                if f < pbest_fitness[i]:
                    pbest_fitness[i] = f
                    pbest[i] = particles[i].copy()
                if f < gbest_fitness:
                    gbest_fitness = f
                    gbest = particles[i].copy()

        # --- Reconstruct with optimal thresholds ---
        denoised = self._reconstruct(cA, detail_list, gbest)
        return denoised[:len(signal)]


# ═══════════════════════════════════════════════════════════════════════════
#  Channel Buffer — manages per-channel sliding window + denoising
# ═══════════════════════════════════════════════════════════════════════════

class ChannelBuffer:
    """Sliding window buffer for a single signal channel."""

    def __init__(self, size, denoiser):
        self.size = size
        self.denoiser = denoiser
        self.buffer = []
        self.denoised_value = 0.0

    def push(self, value):
        """Add a sample and update the denoised estimate."""
        self.buffer.append(value)
        if len(self.buffer) > self.size:
            self.buffer.pop(0)

        if len(self.buffer) >= 8:  # Minimum for 3-level DWT
            arr = np.array(self.buffer, dtype=np.float64)
            denoised = self.denoiser.denoise(arr)
            self.denoised_value = float(denoised[-1])
        else:
            self.denoised_value = value

        return self.denoised_value


# ═══════════════════════════════════════════════════════════════════════════
#  ROS 2 Quantum Filter Node
# ═══════════════════════════════════════════════════════════════════════════

class QuantumFilterNode(Node):
    """
    ROS 2 node that applies quantum-inspired wavelet denoising
    to sensor topics before they reach the EKF.
    """

    def __init__(self):
        super().__init__('quantum_filter_node')

        # --- Parameters ---
        self.declare_parameter('imu_buffer_size', 64)
        self.declare_parameter('gps_buffer_size', 32)
        self.declare_parameter('odom_buffer_size', 64)
        self.declare_parameter('visual_odom_buffer_size', 64)
        self.declare_parameter('lidar_odom_buffer_size', 64)
        self.declare_parameter('wavelet', 'db4')
        self.declare_parameter('dwt_level', 3)
        self.declare_parameter('qpso_particles', 10)
        self.declare_parameter('qpso_iterations', 8)
        self.declare_parameter('alpha', 0.7)

        imu_buf = self.get_parameter('imu_buffer_size').value
        gps_buf = self.get_parameter('gps_buffer_size').value
        odom_buf = self.get_parameter('odom_buffer_size').value
        vo_buf = self.get_parameter('visual_odom_buffer_size').value
        lo_buf = self.get_parameter('lidar_odom_buffer_size').value
        wavelet = self.get_parameter('wavelet').value
        level = self.get_parameter('dwt_level').value
        particles = self.get_parameter('qpso_particles').value
        iterations = self.get_parameter('qpso_iterations').value
        alpha = self.get_parameter('alpha').value

        self.get_logger().info(
            f'Quantum Filter: wavelet={wavelet}, level={level}, '
            f'particles={particles}, iter={iterations}, alpha={alpha}'
        )

        # Create denoisers
        denoiser_imu = QPSOWaveletDenoiser(wavelet, level, particles, iterations, alpha)
        denoiser_gps = QPSOWaveletDenoiser(wavelet, level, particles, iterations, alpha)
        denoiser_odom = QPSOWaveletDenoiser(wavelet, level, particles, iterations, alpha)
        denoiser_vo = QPSOWaveletDenoiser(wavelet, level, particles, iterations, alpha)
        denoiser_lo = QPSOWaveletDenoiser(wavelet, level, particles, iterations, alpha)

        # --- IMU Channel Buffers ---
        # Denoise: accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
        self.imu_channels = {
            'ax': ChannelBuffer(imu_buf, denoiser_imu),
            'ay': ChannelBuffer(imu_buf, denoiser_imu),
            'az': ChannelBuffer(imu_buf, denoiser_imu),
            'gx': ChannelBuffer(imu_buf, denoiser_imu),
            'gy': ChannelBuffer(imu_buf, denoiser_imu),
            'gz': ChannelBuffer(imu_buf, denoiser_imu),
        }

        # --- GPS Channel Buffers ---
        self.gps_channels = {
            'lat': ChannelBuffer(gps_buf, denoiser_gps),
            'lon': ChannelBuffer(gps_buf, denoiser_gps),
            'alt': ChannelBuffer(gps_buf, denoiser_gps),
        }

        # --- Wheel Odom: PASSTHROUGH (no filtering) ---
        # Analysis showed QF adds -0.2% noise to wheel odom (makes it worse).
        # We keep the subscriber → publisher relay so /filtered topic exists,
        # but the data passes through unmodified.
        # (odom channel buffers removed — odom_buf and denoiser_odom unused)

        # --- Visual Odom Channel Buffers ---
        self.vo_channels = {
            'x': ChannelBuffer(vo_buf, denoiser_vo),
            'y': ChannelBuffer(vo_buf, denoiser_vo),
            'z': ChannelBuffer(vo_buf, denoiser_vo),
            'vx': ChannelBuffer(vo_buf, denoiser_vo),
            'vyaw': ChannelBuffer(vo_buf, denoiser_vo),
        }

        # --- LiDAR Odom Channel Buffers ---
        self.lo_channels = {
            'x': ChannelBuffer(lo_buf, denoiser_lo),
            'y': ChannelBuffer(lo_buf, denoiser_lo),
            'z': ChannelBuffer(lo_buf, denoiser_lo),
            'vx': ChannelBuffer(lo_buf, denoiser_lo),
            'vyaw': ChannelBuffer(lo_buf, denoiser_lo),
        }

        # --- Timing stats ---
        self.imu_times = []
        self.gps_times = []
        self.odom_times = []
        self.vo_times = []
        self.lo_times = []

        # --- QoS ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- Subscribers ---
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_cb, sensor_qos)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_cb, sensor_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self.odom_cb, sensor_qos)
        self.vo_sub = self.create_subscription(
            Odometry, '/odometry/visual', self.vo_cb, sensor_qos)
        self.lo_sub = self.create_subscription(
            Odometry, '/odometry/lidar', self.lo_cb, sensor_qos)

        # --- Publishers ---
        self.imu_pub = self.create_publisher(Imu, '/imu/data/filtered', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix/filtered', 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/diff_drive_controller/odom/filtered', 10)
        self.vo_pub = self.create_publisher(
            Odometry, '/odometry/visual/filtered', 10)
        self.lo_pub = self.create_publisher(
            Odometry, '/odometry/lidar/filtered', 10)

        # --- Periodic stats timer (every 30s) ---
        self.create_timer(30.0, self.log_stats)

        self.get_logger().info('Quantum Filter Node started ✓')

    # ─── IMU callback ────────────────────────────────────────────────────
    def imu_cb(self, msg: Imu):
        t0 = time.perf_counter()

        out = copy.deepcopy(msg)
        out.linear_acceleration.x = self.imu_channels['ax'].push(msg.linear_acceleration.x)
        out.linear_acceleration.y = self.imu_channels['ay'].push(msg.linear_acceleration.y)
        out.linear_acceleration.z = self.imu_channels['az'].push(msg.linear_acceleration.z)
        out.angular_velocity.x = self.imu_channels['gx'].push(msg.angular_velocity.x)
        out.angular_velocity.y = self.imu_channels['gy'].push(msg.angular_velocity.y)
        out.angular_velocity.z = self.imu_channels['gz'].push(msg.angular_velocity.z)

        self.imu_pub.publish(out)
        self.imu_times.append(time.perf_counter() - t0)

    # ─── GPS callback ────────────────────────────────────────────────────
    def gps_cb(self, msg: NavSatFix):
        t0 = time.perf_counter()

        out = copy.deepcopy(msg)
        out.latitude = self.gps_channels['lat'].push(msg.latitude)
        out.longitude = self.gps_channels['lon'].push(msg.longitude)
        out.altitude = self.gps_channels['alt'].push(msg.altitude)

        self.gps_pub.publish(out)
        self.gps_times.append(time.perf_counter() - t0)

    # ─── Wheel Odom callback (PASSTHROUGH — QF hurts odom by -0.2%) ────
    def odom_cb(self, msg: Odometry):
        t0 = time.perf_counter()
        # Passthrough: republish raw data unchanged on /filtered topic
        self.odom_pub.publish(msg)
        self.odom_times.append(time.perf_counter() - t0)

    # ─── Visual Odom callback ────────────────────────────────────────────
    def vo_cb(self, msg: Odometry):
        t0 = time.perf_counter()

        out = copy.deepcopy(msg)
        out.pose.pose.position.x = self.vo_channels['x'].push(msg.pose.pose.position.x)
        out.pose.pose.position.y = self.vo_channels['y'].push(msg.pose.pose.position.y)
        out.pose.pose.position.z = self.vo_channels['z'].push(msg.pose.pose.position.z)
        out.twist.twist.linear.x = self.vo_channels['vx'].push(msg.twist.twist.linear.x)
        out.twist.twist.angular.z = self.vo_channels['vyaw'].push(msg.twist.twist.angular.z)

        self.vo_pub.publish(out)
        self.vo_times.append(time.perf_counter() - t0)

    # ─── LiDAR Odom callback ─────────────────────────────────────────────
    def lo_cb(self, msg: Odometry):
        t0 = time.perf_counter()

        out = copy.deepcopy(msg)
        out.pose.pose.position.x = self.lo_channels['x'].push(msg.pose.pose.position.x)
        out.pose.pose.position.y = self.lo_channels['y'].push(msg.pose.pose.position.y)
        out.pose.pose.position.z = self.lo_channels['z'].push(msg.pose.pose.position.z)
        out.twist.twist.linear.x = self.lo_channels['vx'].push(msg.twist.twist.linear.x)
        out.twist.twist.angular.z = self.lo_channels['vyaw'].push(msg.twist.twist.angular.z)

        self.lo_pub.publish(out)
        self.lo_times.append(time.perf_counter() - t0)

    # ─── Stats logging ───────────────────────────────────────────────────
    def log_stats(self):
        for name, times in [
            ('IMU', self.imu_times),
            ('GPS', self.gps_times),
            ('Odom', self.odom_times),
            ('VisualOdom', self.vo_times),
            ('LidarOdom', self.lo_times),
        ]:
            if times:
                arr = np.array(times)
                self.get_logger().info(
                    f'QF {name}: n={len(arr)}, '
                    f'mean={arr.mean()*1000:.2f}ms, '
                    f'max={arr.max()*1000:.2f}ms'
                )
        # Reset
        self.imu_times.clear()
        self.gps_times.clear()
        self.odom_times.clear()
        self.vo_times.clear()
        self.lo_times.clear()


def main(args=None):
    rclpy.init(args=args)
    node = QuantumFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
