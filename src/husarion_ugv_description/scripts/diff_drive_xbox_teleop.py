#!/usr/bin/env python3

from pathlib import Path
import csv

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class DiffDriveXboxTeleop(Node):
    """Xbox joystick teleop for differential drive + optional actuator and data logging."""

    def __init__(self):
        super().__init__("diff_drive_xbox_teleop")

        # -------------------- Parameters --------------------
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("actuator_topic", "/actuator/command")

        self.declare_parameter("max_linear_speed", 1.5)
        self.declare_parameter("max_angular_speed", 2.0)
        self.declare_parameter("scale_linear", 1.0)
        self.declare_parameter("scale_angular", 1.0)
        self.declare_parameter("deadzone", 0.10)

        # Modular speed controls
        self.declare_parameter("slow_mode_scale", 0.30)
        self.declare_parameter("normal_mode_scale", 1.00)
        self.declare_parameter("turbo_mode_scale", 1.50)

        # Optional runtime speed stepping (D-pad left/right)
        self.declare_parameter("enable_speed_steps", True)
        self.declare_parameter("speed_step", 0.10)
        self.declare_parameter("min_speed_scale", 0.30)
        self.declare_parameter("max_speed_scale", 1.20)

        # Safety
        self.declare_parameter("watchdog_timeout_sec", 1.0)
        self.declare_parameter("watchdog_period_sec", 0.5)

        # Data collection
        self.declare_parameter("enable_data_logging", False)
        self.declare_parameter("data_log_dir", "/home/rosdata")
        self.declare_parameter("data_log_file", "teleop_log.csv")

        self.joy_topic = self.get_parameter("joy_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.actuator_topic = self.get_parameter("actuator_topic").value

        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.scale_linear = float(self.get_parameter("scale_linear").value)
        self.scale_angular = float(self.get_parameter("scale_angular").value)
        self.deadzone = float(self.get_parameter("deadzone").value)

        self.slow_mode_scale = float(self.get_parameter("slow_mode_scale").value)
        self.normal_mode_scale = float(self.get_parameter("normal_mode_scale").value)
        self.turbo_mode_scale = float(self.get_parameter("turbo_mode_scale").value)

        self.enable_speed_steps = bool(self.get_parameter("enable_speed_steps").value)
        self.speed_step = float(self.get_parameter("speed_step").value)
        self.min_speed_scale = float(self.get_parameter("min_speed_scale").value)
        self.max_speed_scale = float(self.get_parameter("max_speed_scale").value)
        self.user_speed_scale = self.normal_mode_scale

        self.watchdog_timeout_sec = float(self.get_parameter("watchdog_timeout_sec").value)
        self.watchdog_period_sec = float(self.get_parameter("watchdog_period_sec").value)

        self.enable_data_logging = bool(self.get_parameter("enable_data_logging").value)
        self.data_log_dir = Path(self.get_parameter("data_log_dir").value)
        self.data_log_file = self.get_parameter("data_log_file").value

        # -------------------- Comms --------------------
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.actuator_pub = self.create_publisher(String, self.actuator_topic, 10)
        self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)

        # -------------------- Controller map --------------------
        self.AXIS_LEFT_Y = 1
        self.AXIS_RIGHT_X = 3
        self.AXIS_LT = 2
        self.AXIS_RT = 5
        self.AXIS_DPAD_H = 6
        self.AXIS_DPAD_V = 7

        self.BUTTON_A = 0
        self.BUTTON_B = 1
        self.BUTTON_X = 2
        self.BUTTON_Y = 3
        self.BUTTON_LB = 4
        self.BUTTON_RB = 5

        # -------------------- State --------------------
        self.enabled = True
        self.last_joy_time = self.get_clock().now()

        self.last_a = 0
        self.last_b = 0
        self.last_x = 0
        self.last_y = 0
        self.last_lb = 0
        self.last_rb = 0
        self.last_dpad_h = 0.0

        self.actuator_running = False
        self.actuator_direction = "stopped"
        self.dpad_control_active = False

        self.create_timer(self.watchdog_period_sec, self.watchdog_callback)

        # -------------------- Data logger --------------------
        self._log_handle = None
        self._log_writer = None
        if self.enable_data_logging:
            try:
                self._init_logger()
            except Exception as exc:
                self.enable_data_logging = False
                self.get_logger().warn(
                    f"Data logging disabled (logger init failed): {exc}"
                )

        self.get_logger().info("DiffDrive Xbox Teleop started")
        self.get_logger().info(f"joy_topic={self.joy_topic}, cmd_vel_topic={self.cmd_vel_topic}")
        self.get_logger().info(
            f"speed scales: slow={self.slow_mode_scale:.2f}, normal={self.normal_mode_scale:.2f}, turbo={self.turbo_mode_scale:.2f}"
        )
        if self.enable_speed_steps:
            self.get_logger().info(
                "D-pad LEFT/RIGHT changes base speed scale "
                f"in steps of {self.speed_step:.2f}"
            )

    def _init_logger(self):
        self.data_log_dir.mkdir(parents=True, exist_ok=True)
        log_path = self.data_log_dir / self.data_log_file
        self._log_handle = open(log_path, "a", newline="")
        self._log_writer = csv.writer(self._log_handle)
        if self._log_handle.tell() == 0:
            self._log_writer.writerow([
                "time_sec",
                "enabled",
                "speed_mode",
                "user_speed_scale",
                "linear_x",
                "angular_z",
                "axis_left_y",
                "axis_right_x",
                "axis_lt",
                "axis_rt",
                "dpad_h",
                "dpad_v",
                "actuator_direction",
            ])
        self.get_logger().info(f"Logging teleop data to: {log_path}")

    def _safe_axis(self, msg: Joy, idx: int, default: float = 0.0) -> float:
        return float(msg.axes[idx]) if len(msg.axes) > idx else default

    def _safe_button(self, msg: Joy, idx: int, default: int = 0) -> int:
        return int(msg.buttons[idx]) if len(msg.buttons) > idx else default

    def apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def send_actuator_command(self, command: str):
        msg = String()
        msg.data = command
        self.actuator_pub.publish(msg)

    def set_speed_scale(self, new_scale: float):
        self.user_speed_scale = max(self.min_speed_scale, min(self.max_speed_scale, new_scale))
        self.get_logger().info(f"Base speed scale set to {self.user_speed_scale:.2f}")

    def joy_callback(self, msg: Joy):
        self.last_joy_time = self.get_clock().now()

        # Buttons
        a = self._safe_button(msg, self.BUTTON_A)
        b = self._safe_button(msg, self.BUTTON_B)
        x_btn = self._safe_button(msg, self.BUTTON_X)
        y_btn = self._safe_button(msg, self.BUTTON_Y)
        lb = self._safe_button(msg, self.BUTTON_LB)
        rb = self._safe_button(msg, self.BUTTON_RB)

        # Axes
        left_y = self.apply_deadzone(self._safe_axis(msg, self.AXIS_LEFT_Y))
        right_x = self.apply_deadzone(self._safe_axis(msg, self.AXIS_RIGHT_X))
        lt_raw = self._safe_axis(msg, self.AXIS_LT, 1.0)
        rt_raw = self._safe_axis(msg, self.AXIS_RT, 1.0)
        dpad_h = self._safe_axis(msg, self.AXIS_DPAD_H)
        dpad_v = self._safe_axis(msg, self.AXIS_DPAD_V)

        # Emergency stop
        if a and not self.last_a:
            self.emergency_stop()
            self.last_a = a
            return
        self.last_a = a

        # Enable/disable drive
        if b and not self.last_b:
            self.enabled = not self.enabled
            self.get_logger().info(f"Drive {'ENABLED' if self.enabled else 'DISABLED'}")
        self.last_b = b

        # Speed step controls (D-pad left/right rising edge)
        if self.enable_speed_steps:
            if dpad_h > 0.5 and self.last_dpad_h <= 0.5:
                self.set_speed_scale(self.user_speed_scale + self.speed_step)
            elif dpad_h < -0.5 and self.last_dpad_h >= -0.5:
                self.set_speed_scale(self.user_speed_scale - self.speed_step)
            self.last_dpad_h = dpad_h

        # Actuator: D-pad hold mode has priority
        if dpad_v > 0.5:
            if not self.actuator_running or self.actuator_direction != "up" or not self.dpad_control_active:
                self.send_actuator_command("up")
                self.actuator_running = True
                self.actuator_direction = "up"
                self.dpad_control_active = True
        elif dpad_v < -0.5:
            if not self.actuator_running or self.actuator_direction != "down" or not self.dpad_control_active:
                self.send_actuator_command("down")
                self.actuator_running = True
                self.actuator_direction = "down"
                self.dpad_control_active = True
        else:
            if self.dpad_control_active:
                self.send_actuator_command("stop")
                self.actuator_running = False
                self.actuator_direction = "stopped"
                self.dpad_control_active = False

        # Toggle mode only when D-pad is not active
        if not self.dpad_control_active:
            if y_btn and not self.last_y:
                self.send_actuator_command("up")
                self.actuator_running = True
                self.actuator_direction = "up"
            if x_btn and not self.last_x:
                self.send_actuator_command("down")
                self.actuator_running = True
                self.actuator_direction = "down"
            if (lb and not self.last_lb) or (rb and not self.last_rb):
                self.send_actuator_command("stop")
                self.actuator_running = False
                self.actuator_direction = "stopped"

        self.last_x = x_btn
        self.last_y = y_btn
        self.last_lb = lb
        self.last_rb = rb

        if not self.enabled:
            return

        # Trigger mode selection
        lt_pressed = (1.0 - lt_raw) / 2.0
        rt_pressed = (1.0 - rt_raw) / 2.0

        mode_scale = self.user_speed_scale
        speed_mode = "NORMAL"
        if lt_pressed > 0.1:
            mode_scale *= self.slow_mode_scale
            speed_mode = "SLOW"
        elif rt_pressed > 0.1:
            mode_scale *= self.turbo_mode_scale
            speed_mode = "TURBO"

        twist = Twist()
        twist.linear.x = left_y * self.max_linear_speed * self.scale_linear * mode_scale
        twist.angular.z = -right_x * self.max_angular_speed * self.scale_angular * mode_scale

        self.cmd_vel_pub.publish(twist)

        if self.enable_data_logging and self._log_writer is not None:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            self._log_writer.writerow([
                f"{now_sec:.6f}",
                int(self.enabled),
                speed_mode,
                f"{self.user_speed_scale:.3f}",
                f"{twist.linear.x:.6f}",
                f"{twist.angular.z:.6f}",
                f"{left_y:.6f}",
                f"{right_x:.6f}",
                f"{lt_raw:.6f}",
                f"{rt_raw:.6f}",
                f"{dpad_h:.6f}",
                f"{dpad_v:.6f}",
                self.actuator_direction,
            ])

        if abs(twist.linear.x) > 0.01 or abs(twist.angular.z) > 0.01:
            self.get_logger().info(
                f"[{speed_mode}] linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}",
                throttle_duration_sec=0.5,
            )

    def emergency_stop(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.send_actuator_command("stop")
        self.enabled = False
        self.actuator_running = False
        self.actuator_direction = "stopped"
        self.dpad_control_active = False
        self.get_logger().warn("Emergency stop activated")

    def watchdog_callback(self):
        dt = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if dt > self.watchdog_timeout_sec:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            if self.actuator_running:
                self.send_actuator_command("stop")
                self.actuator_running = False
                self.actuator_direction = "stopped"
                self.dpad_control_active = False

    def destroy_node(self):
        if self._log_handle is not None:
            self._log_handle.flush()
            self._log_handle.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveXboxTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
