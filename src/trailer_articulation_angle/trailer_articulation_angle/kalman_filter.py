#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class SimpleKalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.x_hat = 0.0  # Initial state estimate
        self.estimated_measurement_variance = 1.0  # Initial estimate variance

    def update(self, measurement):
        # Prediction update: assume the state does not change
        innovation = measurement - self.x_hat
        innovation_covariance = self.estimated_measurement_variance + self.measurement_variance
        
        # Kalman Gain
        kalman_gain = self.estimated_measurement_variance / innovation_covariance
        
        # Update the estimate with the new measurement
        self.x_hat += kalman_gain * innovation
        
        # Update the estimated variance
        self.estimated_measurement_variance = (1 - kalman_gain) * self.estimated_measurement_variance
        
        return self.x_hat

class ArticulationAngleFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter')
        self.subscription_markers = self.create_subscription(
            Float64,
            'articulation_angle/markers',
            self.listener_callback_markers,
            10)
        self.kalman_filter_publisher = self.create_publisher(Float64, 'articulation_angle/filtered', 10)
        # Initialize the Kalman filter with appropriate variances for the markers
        self.kalman_filter = SimpleKalmanFilter(1e-5, 1e-5)

    def listener_callback_markers(self, msg):
        filtered_value = self.kalman_filter.update(msg.data)
        self.get_logger().info(f'Filtered Articulation Angle: {filtered_value}')
        # Publish the filtered value
        filtered_msg = Float64()
        filtered_msg.data = float(filtered_value)
        self.kalman_filter_publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    articulation_angle_filter = ArticulationAngleFilter()
    rclpy.spin(articulation_angle_filter)
    articulation_angle_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
