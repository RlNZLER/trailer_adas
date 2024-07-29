#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist

class TeleopController(Node):
    
    def __init__(self):
        super().__init__('teleop_controller')
        
        self.declare_parameter('wheel_radius', 0.5715)
        self.declare_parameter('wheel_separation', 1.225)
        
        self.wheel_radius_ = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter('wheel_separation').get_parameter_value().double_value
        
        self.get_logger().info("Using wheel radius: " + str(self.wheel_radius_))
        self.get_logger().info("Using wheel separation: " + str(self.wheel_separation_))
        
        # Publisher to Ackermann steering controller reference topic
        self.ackermann_cmd_pub_ = self.create_publisher(TwistStamped, "/ackermann_steering_controller/reference", 10)
        
        # Subscriber to joystick and keyboard input
        self.joy_sub_ = self.create_subscription(Twist, "trailer_controller/cmd_vel", self.cmd_vel_callback, 10)
        
        # Initialize speed variables
        self.current_speed = 0.0
        self.max_speed = 15.0
        self.speed_increment = 0.5
        self.speed_decrement = 1.0
        self.angular_z = 0.0
        self.direction = 1.0  # 1 for forward, -1 for reverse
        self.parked = False  # True if the car is in park mode
        self.previous_park_button_state = False  # To track the previous state of the park button

    def cmd_vel_callback(self, msg):
        # Process the incoming Twist message
        self.current_speed = msg.linear.x
        self.angular_z = msg.angular.z

        # Create and publish TwistStamped message
        ackermann_cmd = TwistStamped()
        ackermann_cmd.header.stamp = self.get_clock().now().to_msg()
        ackermann_cmd.twist.linear.x = float(self.current_speed * self.direction) if not self.parked else 0.0
        ackermann_cmd.twist.angular.z = -float(self.angular_z) if self.direction == 1 else float(self.angular_z)
        self.ackermann_cmd_pub_.publish(ackermann_cmd)
        
def main(args=None):
    rclpy.init(args=args)
    teleop_controller = TeleopController()
    rclpy.spin(teleop_controller)
    teleop_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
