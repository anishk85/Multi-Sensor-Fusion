#!/usr/bin/env python3

import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
Husarion Lynx Custom Teleop
---------------------------
Move around:
   w
a  s  d

Change Speeds:
q / z : increase/decrease linear speed by 10%
e / c : increase/decrease angular speed by 10%

Space/k : force stop
CTRL-C  : quit
"""

moveBindings = {
    'w': (1, 0),
    's': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
    ' ': (0, 0),
    'k': (0, 0),
}

speedBindings = {
    'q': (1.1, 1.0),
    'z': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}

class LynxTeleop(Node):
    def __init__(self):
        super().__init__('custom_teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Custom Lynx Teleop Initialized')

    def publish_twist(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = float(linear_vel)
        twist.angular.z = float(angular_vel)
        self.publisher_.publish(twist)

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = LynxTeleop()
    
    speed = 0.5   # Base linear speed
    turn = 1.0    # Base angular speed
    x = 0.0
    th = 0.0
    
    print(msg)
    
    try:
        while rclpy.ok():
            key = getKey(settings)
            
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                
                # Cap the maximum speeds
                speed = min(1.5, speed)
                turn = min(3.0, turn)
            else:
                if key == '\x03':  # CTRL-C
                    break
                    
            linear_vel = x * speed
            angular_vel = th * turn
            
            if key:
                sys.stdout.write(f"\rMax Speed: {speed:.2f} m/s | Max Turn: {turn:.2f} rad/s || CMD: \033[92m{linear_vel:.2f}\033[0m m/s \033[92m{angular_vel:.2f}\033[0m rad/s   ")
                sys.stdout.flush()

            node.publish_twist(linear_vel, angular_vel)
            rclpy.spin_once(node, timeout_sec=0)

    except Exception as e:
        print(e)
    finally:
        node.publish_twist(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
