#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import sys
import os

if sys.platform == 'win32':
    import msvcrt
else:
    import tty
    import termios

def getKey(settings):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

message = """
Twist Control Node for Ackermann Steering
-----------------------------------------
Moving around:
            ^
            w
    <   a   s   d   >
-----------------------------------------
w/s : increase/decrease linear speed (x)
a/d : increase/decrease angular speed (z)
r   : reset steering to straight
q   : stop and quit
"""

class TwistControl(Node):
    def __init__(self):
        super().__init__('Twist_Control')
        self.publisher_ = self.create_publisher(Twist, '/ackermann_steering_controller/reference_unstamped', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        print(message)

    def timer_callback(self):
        msg = Twist()
        settings = termios.tcgetattr(sys.stdin.fileno()) if sys.platform != 'win32' else None
        key = getKey(settings=settings)

        if key.lower() == 'w':
            self.linear_speed += 0.1
        elif key.lower() == 's':
            self.linear_speed -= 0.1
        elif key.lower() == 'a':
            self.angular_speed += math.pi/36
        elif key.lower() == 'd':
            self.angular_speed -= math.pi/36
        elif key.lower() == 'r':
            self.angular_speed = 0.0
        elif key.lower() == 'q':
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            msg.linear.x = self.linear_speed
            msg.angular.z = self.angular_speed
            self.publisher_.publish(msg)
            print(f"Exiting: linear_speed={self.linear_speed}, angular_speed={self.angular_speed}")
            sys.exit(0)

        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)
        print(f"linear_speed={self.linear_speed}, angular_speed={self.angular_speed}")

def main(args=None):
    rclpy.init(args=args)
    twist_control = TwistControl()
    rclpy.spin(twist_control)
    twist_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
