#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
import numpy as np
import time
import threading

class RangeProcessorNode(Node):
    def __init__(self):
        super().__init__('range_processor_node')
        
        self.create_subscription(Range, 'range/u1', self.range_callback_u1, 10)
        self.create_subscription(Range, 'range/u2', self.range_callback_u2, 10)
        self.create_subscription(Range, 'range/u3', self.range_callback_u3, 10)
        self.create_subscription(Range, 'range/u4', self.range_callback_u4, 10)
        
        # Publish the articulation angle in radians
        self.range_art_angle_pub_ = self.create_publisher(Float64, 'articulation_angle/range', 10)
        
        # Publisher for the time duration from receiving the range data to calculating the articulation angle
        self.time_publisher = self.create_publisher(Float64, 'articulation_angle/range_delay', 10)

        self.x_u1 = -0.6125
        self.x_u2 = -0.26
        self.x_u3 = 0.26
        self.x_u4 = 0.6125
        self.points_u1 = None
        self.points_u2 = None
        self.points_u3 = None
        self.points_u4 = None
        
        self.start_time = None  # To record the start time when the first range message is received
        self.calculation_triggered = False  # To avoid multiple triggers

    def range_callback_u1(self, msg):
        if self.start_time is None:
            self.start_time = time.time()  
        self.points_u1 = (self.x_u1, msg.range)
        self.trigger_calculation_if_ready()

    def range_callback_u2(self, msg):
        if self.start_time is None:
            self.start_time = time.time()  
        self.points_u2 = (self.x_u2, msg.range)
        self.trigger_calculation_if_ready()

    def range_callback_u3(self, msg):
        if self.start_time is None:
            self.start_time = time.time()  
        self.points_u3 = (self.x_u3, msg.range)
        self.trigger_calculation_if_ready()

    def range_callback_u4(self, msg):
        if self.start_time is None:
            self.start_time = time.time()  
        self.points_u4 = (self.x_u4, msg.range)
        self.trigger_calculation_if_ready()

    def trigger_calculation_if_ready(self):
        # Check how many valid points we have
        available_points = [self.points_u1, self.points_u2, self.points_u3, self.points_u4]
        filtered_points = [point for point in available_points if point is not None and point[1] <= 2]
        
        # If we have at least two valid points, initiate a delay before calculating
        if len(filtered_points) >= 2 and not self.calculation_triggered:
            self.calculation_triggered = True  # Avoid multiple triggers
            threading.Timer(0.05, self.calculate_articulation_angle, args=[filtered_points]).start()

    def calculate_articulation_angle(self, filtered_points):
        available_points = [self.points_u1, self.points_u2, self.points_u3, self.points_u4]
        # Re-check if more points became available during the delay
        filtered_points = [point for point in available_points if point is not None and point[1] <= 2]

        x_values = np.array([point[0] for point in filtered_points])
        y_values = np.array([point[1] for point in filtered_points])
        
        # Prepare the matrix for lstsq
        A = np.vstack([x_values, np.ones(len(x_values))]).T  # T for transpose
        m, b = np.linalg.lstsq(A, y_values, rcond=None)[0]  # m is slope, b is intercept
        
        # Calculate the angle of the slope in radians
        angle = -np.arctan(m)
            
        # Publish the articulation angle in radians
        msg = Float64()
        msg.data = angle
        self.range_art_angle_pub_.publish(msg)

        # Calculate the time duration from receiving the range values to calculating the angle
        end_time = time.time()
        time_duration = end_time - self.start_time
        
        # Publish the time duration
        time_msg = Float64()
        time_msg.data = time_duration
        self.time_publisher.publish(time_msg)
        
        # Reset start_time and points to None to wait for the next cycle
        self.start_time = None
        self.points_u1 = None
        self.points_u2 = None
        self.points_u3 = None
        self.points_u4 = None
        self.calculation_triggered = False  # Reset trigger


def main(args=None):
    rclpy.init(args=args)
    range_node = RangeProcessorNode()
    rclpy.spin(range_node)
    range_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
