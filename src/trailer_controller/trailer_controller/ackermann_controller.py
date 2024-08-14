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

class AckermannController(Node):
    
    def __init__(self):
        super().__init__('ackermann_controller')
        
        self.speed_scaling_factor = 2.0
        self.declare_parameter('wheel_radius', 0.28575)     # Radius of the wheels.
        self.declare_parameter('wheel_separation', 1.225)   # Distance between the two rear wheels.
        self.declare_parameter('wheel_base', 2.63)          # Distance between front and rear axles.
        
        self.wheel_radius_ = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.wheel_base_ = self.get_parameter('wheel_base').get_parameter_value().double_value
        
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()
        
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        
        self.get_logger().info("Using wheel radius: " + str(self.wheel_radius_))
        self.get_logger().info("Using wheel separation: " + str(self.wheel_separation_))
        self.get_logger().info("Using wheel base: " + str(self.wheel_base_))
        
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.steer_cmd_pub_ = self.create_publisher(Float64MultiArray, "steering_position_controller/commands", 10)
        self.status_pub_ = self.create_publisher(Float64MultiArray, "vehicle_status", 10)
        
        # Publisher to odometry topic
        self.odom_pub_ = self.create_publisher(Odometry, "trailer_controller/odom", 10)
        
        # Subscriber to joystick input
        self.joy_sub_ = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        
        # Subscriber to joint states
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.joint_callback, 10)
        
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0
        
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"
        
    def joy_callback(self, msg):
        # Initialize a variable to determine if the wireless joystick is active
        wireless_joystick_active = False
        # if msg.axes[4] == 1:
        #     wireless_joystick_active = True
        if wireless_joystick_active:
            # Use key 3 for wireless joystick
            linear_velocity = msg.axes[3] * 10
        else:
            # Use key 4 for wired joystick
            linear_velocity = msg.axes[4] * 10

        angular_velocity = msg.axes[0] * 2.2437823071658

        if angular_velocity != 0:
            turning_radius = abs(linear_velocity / angular_velocity)
            
            R_inner = abs(turning_radius - (self.wheel_separation_ / 2))
            R_outer = abs(turning_radius + (self.wheel_separation_ / 2))

            if angular_velocity > 0:  # Turning left
                left_wheel_angle = -atan2(self.wheel_base_, R_inner)
                right_wheel_angle = -atan2(self.wheel_base_, R_outer)
                
                if linear_velocity == 0:
                    V_rear_left = 0.0
                    V_rear_right = 0.0
                elif linear_velocity > 0:
                    V_rear_left = abs(angular_velocity * R_inner)
                    V_rear_right = abs(angular_velocity * R_outer)
                else:
                    V_rear_left = -abs(angular_velocity * R_inner)
                    V_rear_right = -abs(angular_velocity * R_outer)
                    
            else:  # Turning right
                left_wheel_angle = atan2(self.wheel_base_, R_outer)
                right_wheel_angle = atan2(self.wheel_base_, R_inner)
                                
                if linear_velocity == 0:
                    V_rear_left = 0.0
                    V_rear_right = 0.0
                elif linear_velocity > 0:
                    V_rear_right = abs(angular_velocity * R_inner)
                    V_rear_left = abs(angular_velocity * R_outer)
                else:
                    V_rear_right = -abs(angular_velocity * R_inner)
                    V_rear_left = -abs(angular_velocity * R_outer)
                    
        elif linear_velocity != 0 and angular_velocity == 0:
            left_wheel_angle = 0.0
            right_wheel_angle = 0.0
            V_rear_left = V_rear_right = linear_velocity
            
        else:
            left_wheel_angle = 0.0
            right_wheel_angle = 0.0
            V_rear_left = 0.0
            V_rear_right = 0.0

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [V_rear_left / (self.speed_scaling_factor * self.wheel_radius_), V_rear_right / (self.speed_scaling_factor * self.wheel_radius_)]
        self.wheel_cmd_pub_.publish(wheel_speed_msg)

        steer_cmd_msg = Float64MultiArray()
        steer_cmd_msg.data = [left_wheel_angle, right_wheel_angle]
        self.steer_cmd_pub_.publish(steer_cmd_msg)
        
        steering_angle = (left_wheel_angle + right_wheel_angle) / 2 # Average steering angle
        
        # Publish the vehicle status
        vehicle_status_msg = Float64MultiArray()
        vehicle_status_msg.data = [
            linear_velocity,
            angular_velocity,
            steering_angle,  
            left_wheel_angle,
            right_wheel_angle,
            V_rear_left,
            V_rear_right
        ]
        self.status_pub_.publish(vehicle_status_msg)


    def joint_callback(self, msg):
        # Using the correct joint names based on the actual JointState message
        try:
            left_wheel_index = msg.name.index('rear_left_wheel_joint')
            right_wheel_index = msg.name.index('rear_right_wheel_joint')
        except ValueError:
            self.get_logger().error("Left or right wheel joint not found in JointState message")
            return
        
        dp_left = msg.position[left_wheel_index] - self.left_wheel_prev_pos_
        dp_right = msg.position[right_wheel_index] - self.right_wheel_prev_pos_
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
    ackermann_controller = AckermannController()
    rclpy.spin(ackermann_controller)
    ackermann_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
