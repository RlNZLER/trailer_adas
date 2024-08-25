#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ArticulationAngleFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter')
        self.measurement_buffer = [None, None]

        # Kalman filter state
        self.x = np.array([[0], [0]])
        self.P = np.eye(2) * 100
        self.A = np.array([[1, 0.1], [0, 1]])
        self.Q = np.array([[0.2, 0], [0, 0.2]])
        self.H = np.array([[1, 0]])
        self.R = np.array([[0.1]])

        self.subscription_markers = self.create_subscription(
            Float64,
            'articulation_angle/markers',
            self.listener_callback_markers,
            10)
        self.subscription_range = self.create_subscription(
            Float64,
            'articulation_angle/range',
            self.listener_callback_range,
            10)
        self.timer_ = self.create_timer(0.1, self.process_measurements)
        self.kalman_filter_publisher = self.create_publisher(Float64, 'articulation_angle/filtered', 10)
        self.kalman_timestamp_publisher_ = self.create_publisher(Float64, 'articulation_angle/kalman_timestamp', 10)


    def listener_callback_markers(self, msg):
        self.measurement_buffer[0] = msg.data
        # self.get_logger().info(f'Received marker measurement: {msg.data}')

    def listener_callback_range(self, msg):
        self.measurement_buffer[1] = msg.data
        # self.get_logger().info(f'Received range measurement: {msg.data}')

    def process_measurements(self):
        # Only proceed if both measurements are available
        if all(measurement is not None for measurement in self.measurement_buffer):
            self.kalman_predict()

            for i, measurement in enumerate(self.measurement_buffer):
                self.kalman_update(measurement)

            # Reset measurement buffer
            self.measurement_buffer = [None, None]
            
            # Publish timestamp as Float64
            timestamp_msg = Float64()
            timestamp_msg.data = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
            self.kalman_timestamp_publisher_.publish(timestamp_msg) # Publish the timestamp in seconds
            
            # Publish the filtered value
            filtered_value = Float64()
            filtered_value.data = self.x[0, 0]
            self.kalman_filter_publisher.publish(filtered_value)

    def kalman_predict(self):
        # Prediction step
        self.x = np.dot(self.A, self.x)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        # self.get_logger().info(f'Prediction step: x = {self.x.flatten()}, P = {self.P.flatten()}')

    def kalman_update(self, z):
        # Measurement update step
        y = np.array([[z]]) - np.dot(self.H, self.x)  # Innovation
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R  # Innovation covariance
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  # Kalman gain
        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(K, np.dot(self.H, self.P))
        # self.get_logger().info(f'Update step: z = {z}, x = {self.x.flatten()}, P = {self.P.flatten()}, K = {K.flatten()}')

def main(args=None):
    rclpy.init(args=args)
    articulation_angle_filter = ArticulationAngleFilter()
    rclpy.spin(articulation_angle_filter)
    articulation_angle_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
