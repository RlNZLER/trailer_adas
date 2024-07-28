#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

class SimpleController(Node):
    
    def __init__(self):
        super().__init__('simple_controller')
        
        self.declare_parameter('wheel_radius', 0.5715)
        self.declare_parameter('wheel_separation', 1.225)
        
        self.wheel_radius_ = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter('wheel_separation').get_parameter_value().double_value
        
        self.get_logger().info("Using wheel radius: " + str(self.wheel_radius_))
        self.get_logger().info("Using wheel separation: " + str(self.wheel_separation_))
        
        # Publisher to Ackermann steering controller reference topic
        self.ackermann_cmd_pub_ = self.create_publisher(TwistStamped, "/ackermann_steering_controller/reference", 10)
        
        # Subscriber to joystick input
        self.joy_sub_ = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        
        # Initialize speed variables
        self.current_speed = 0.0
        self.max_speed = 15.0
        self.speed_increment = 0.5
        self.speed_decrement = 3.0
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
        
def main(args=None):
    rclpy.init(args=args)
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
