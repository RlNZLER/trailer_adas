#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class AckermannController(Node):
    
    def __init__(self):
        super().__init__('ackermann_controller')
        
        self.declare_parameter('wheel_radius', 0.5715)
        self.declare_parameter('wheel_separation', 1.225)
        
        self.wheel_radius_ = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter('wheel_separation').get_parameter_value().double_value
        
        self.get_logger().info("Using wheel radius: " + str(self.wheel_radius_))
        self.get_logger().info("Using wheel separation: " + str(self.wheel_separation_))
        
        # Change the publisher to publish to the Ackermann controller reference topic
        self.ackermann_cmd_pub_ = self.create_publisher(TwistStamped, "/ackermann_steering_controller/reference", 10)
        
        # Subscribe to the joystick cmd_vel topic
        self.vel_sub_ = self.create_subscription(TwistStamped, "trailer_controller/cmd_vel", self.velCallback, 10)
        
    def velCallback(self, msg):
        # Forward the received TwistStamped message to the Ackermann steering controller
        self.get_logger().info(f"Received TwistStamped: {msg.twist.linear.x}, {msg.twist.angular.z}")
        
        ackermann_cmd = TwistStamped()
        ackermann_cmd.header.stamp = self.get_clock().now().to_msg()  # Add timestamp
        ackermann_cmd.twist.linear.x = msg.twist.linear.x
        ackermann_cmd.twist.angular.z = -msg.twist.angular.z
        self.ackermann_cmd_pub_.publish(ackermann_cmd)


def main(args=None):
    rclpy.init(args=args)
    ackermann_controller = AckermannController()
    rclpy.spin(ackermann_controller)
    ackermann_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
