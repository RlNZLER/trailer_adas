#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from math import cos, sin


class SimpleController(Node):
    
    def __init__(self):
        super().__init__('simple_controller')
        
        self.declare_parameter('wheel_radius', 0.5715)
        self.declare_parameter('wheel_separation', 1.225)
        
        self.wheel_radius_ = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter('wheel_separation').get_parameter_value().double_value
        
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()
        
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        
        
        self.get_logger().info("Using wheel radius: " + str(self.wheel_radius_))
        self.get_logger().info("Using wheel separation: " + str(self.wheel_separation_))
        
        # Publisher to Ackermann steering controller reference topic
        self.ackermann_cmd_pub_ = self.create_publisher(TwistStamped, "/ackermann_steering_controller/reference", 10)
        
        # Subscriber to joystick input
        self.joy_sub_ = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        
        # Subscriber to joint states
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.joint_callback, 10)
        
        # Initialize speed variables
        self.current_speed = 0.0
        self.max_speed = 15.0
        self.speed_increment = 0.5
        self.speed_decrement = 1.0
        self.angular_z = 0.0
        self.direction = 1.0  # 1 for forward, -1 for reverse
        self.parked = False  # True if the car is in park mode
        self.previous_park_button_state = False  # To track the previous state of the park button

    def joy_callback(self, msg):
        # Button 5 for accelerating
        if msg.buttons[5] and not self.parked:  
            self.current_speed += self.speed_increment
            if self.current_speed > self.max_speed:
                self.current_speed = self.max_speed

        # Button 4 for braking
        if msg.buttons[4] and not self.parked:  
            self.current_speed -= self.speed_decrement
            if self.current_speed < 0:
                self.current_speed = 0

        # Button 1 for switching direction (forward/reverse)
        if msg.buttons[1]:  
            self.direction *= -1
            self.get_logger().info(f"Direction switched to {'forward' if self.direction == 1 else 'reverse'}")

        # Button 2 for parking - toggle behavior
        if msg.buttons[2] and not self.previous_park_button_state:  
            self.parked = not self.parked
            self.current_speed = 0.0 if self.parked else self.current_speed
            self.get_logger().info(f"Car is {'parked' if self.parked else 'unparked'}")
        self.previous_park_button_state = msg.buttons[2]

        # Axis 0 for steering (left/right)
        self.angular_z = msg.axes[0] * 0.5  

        # Create and publish TwistStamped message
        ackermann_cmd = TwistStamped()
        ackermann_cmd.header.stamp = self.get_clock().now().to_msg()
        ackermann_cmd.twist.linear.x = float(self.current_speed * self.direction) if not self.parked else 0.0
        ackermann_cmd.twist.angular.z = -float(self.angular_z) if self.direction == 1 else float(self.angular_z)
        self.ackermann_cmd_pub_.publish(ackermann_cmd)
        
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
        
        self.get_logger().info(f"Linear velocity: {linear}, Angular velocity: {angular}")
        self.get_logger().info(f"Position: ({self.x_}, {self.y_}), Orientation: {self.theta_}")
        
        
def main(args=None):
    rclpy.init(args=args)
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
