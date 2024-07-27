#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np

class SimpleController(Node):
    
    def __init__(self):
        super().__init__('ackermann_controller')
        
        self.declare_parameter('wheel_radius', 0.5715)
        self.declare_parameter('wheel_separation', 1.225)
        self.declare_parameter('wheel_base', 1.5)  # Distance between front and rear axles
        
        self.wheel_radius_ = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.wheel_base_ = self.get_parameter('wheel_base').get_parameter_value().double_value
        
        self.get_logger().info("Using wheel radius: " + str(self.wheel_radius_))
        self.get_logger().info("Using wheel separation: " + str(self.wheel_separation_))
        self.get_logger().info("Using wheel base: " + str(self.wheel_base_))
        
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.steer_cmd_pub_ = self.create_publisher(Float64MultiArray, "steering_position_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "trailer_controller/cmd_vel", self.velCallback, 10)
        
        self.get_logger().info("Node initialized and ready to control the vehicle.") 
        
        
    def velCallback(self, msg):
        linear_velocity = msg.twist.linear.x
        angular_velocity = msg.twist.angular.z
        
        if angular_velocity != 0:
            turning_radius = linear_velocity / angular_velocity
            if angular_velocity > 0:  # Turning left
                left_wheel_angle = np.arctan2(self.wheel_base_, turning_radius - (self.wheel_separation_ / 2))
                right_wheel_angle = np.arctan2(self.wheel_base_, turning_radius + (self.wheel_separation_ / 2))
                
                R_inner = turning_radius - (self.wheel_separation_ / 2)
                R_outer = turning_radius + (self.wheel_separation_ / 2)
            else:  # Turning right
                left_wheel_angle = np.arctan2(self.wheel_base_, turning_radius + (self.wheel_separation_ / 2))
                right_wheel_angle = np.arctan2(self.wheel_base_, turning_radius - (self.wheel_separation_ / 2))
                
                R_inner = turning_radius + (self.wheel_separation_ / 2)
                R_outer = turning_radius - (self.wheel_separation_ / 2)
            
            V_inner = angular_velocity * R_inner
            V_outer = angular_velocity * R_outer
        else:
            left_wheel_angle = 0.0
            right_wheel_angle = 0.0
            V_inner = linear_velocity
            V_outer = linear_velocity

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [V_inner / self.wheel_radius_, V_outer / self.wheel_radius_]
        self.wheel_cmd_pub_.publish(wheel_speed_msg)
        
        steer_cmd_msg = Float64MultiArray()
        steer_cmd_msg.data = [left_wheel_angle, right_wheel_angle]
        self.steer_cmd_pub_.publish(steer_cmd_msg)
        

def main(args=None):
    rclpy.init()
    ackermann_controller = SimpleController()
    rclpy.spin(ackermann_controller)
    ackermann_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
