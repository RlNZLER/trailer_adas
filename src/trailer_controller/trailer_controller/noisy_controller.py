#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped, TransformStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from nav_msgs.msg import Odometry
from math import cos, sin, atan2
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import numpy as np

class NoisyController(Node):
    
    def __init__(self):
        super().__init__('noisy_controller')
        
        self.declare_parameter('wheel_radius', 0.5715)
        self.declare_parameter('wheel_separation', 1.225)
        self.declare_parameter('wheel_base', 2.63)  # Distance between front and rear axles
        
        self.wheel_radius_ = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.wheel_base_ = self.get_parameter('wheel_base').get_parameter_value().double_value
        
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()
        
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        
        # self.get_logger().info("Using wheel radius: " + str(self.wheel_radius_))
        # self.get_logger().info("Using wheel separation: " + str(self.wheel_separation_))
        # self.get_logger().info("Using wheel base: " + str(self.wheel_base_))
        
        # Publisher to noisy odometry topic
        self.odom_pub_ = self.create_publisher(Odometry, "trailer_controller/odom_noisy", 10)
        
        # Subscriber to joint states
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.joint_callback, 10)
        
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint_ekf"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0
        
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint_noisy"


    def joint_callback(self, msg):
        # Using the correct joint names based on the actual JointState message
        try:
            left_wheel_index = msg.name.index('rear_left_wheel_joint')
            right_wheel_index = msg.name.index('rear_right_wheel_joint')
        except ValueError:
            self.get_logger().error("Left or right wheel joint not found in JointState message")
            return
        
        wheel_encoder_left = msg.position[left_wheel_index] + np.random.normal(0, 0.005)
        wheel_encoder_right = msg.position[right_wheel_index] + np.random.normal(0, 0.005)
        dp_left = wheel_encoder_left - self.left_wheel_prev_pos_
        dp_right = wheel_encoder_right - self.right_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_
        
        self.left_wheel_prev_pos_ = msg.position[left_wheel_index]
        self.right_wheel_prev_pos_ = msg.position[right_wheel_index]
        self.prev_time_ = Time.from_msg(msg.header.stamp)
        
        if dt.nanoseconds == 0:
            self.get_logger().error("Delta time is zero, skipping calculation")
            return
        
        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)
        
        linear = self.wheel_radius_ * (fi_right + fi_left) / 2
        angular = ( self.wheel_radius_ * (fi_right - fi_left) ) / self.wheel_separation_
        
        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left ) / 2
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_
        
        self.x_ += d_s * cos(self.theta_)
        self.y_ += d_s * sin(self.theta_)
        self.theta_ += d_theta
        
        q = quaternion_from_euler(0.0, 0.0, self.theta_)
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        
        self.odom_pub_.publish(self.odom_msg_)
        self.br_.sendTransform(self.transform_stamped_)
        
        # self.get_logger().info(f"Linear velocity: {linear}, Angular velocity: {angular}")
        # self.get_logger().info(f"Position: ({self.x_}, {self.y_}), Orientation: {self.theta_}")
        
        
def main(args=None):
    rclpy.init(args=args)
    noisy_controller = NoisyController()
    rclpy.spin(noisy_controller)
    noisy_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
