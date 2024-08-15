import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class KalmanFilterMultiMeasurement:
    def __init__(self, process_variance, measurement_variances):
        self.process_variance = process_variance
        self.measurement_variances = measurement_variances
        self.x_hat = 0.0
        self.estimated_measurement_variance = 1.0

    def update(self, measurements):
        total_kalman_gain = 0
        total_innovation = 0
        for i, measurement in enumerate(measurements):
            if measurement is not None:
                measurement_variance = self.measurement_variances[i]
                innovation = measurement - self.x_hat
                innovation_covariance = self.estimated_measurement_variance + measurement_variance
                kalman_gain = self.estimated_measurement_variance / innovation_covariance
                total_kalman_gain += kalman_gain
                total_innovation += kalman_gain * innovation

        self.x_hat += total_innovation
        if total_kalman_gain > 0:
            self.estimated_measurement_variance = (1 - total_kalman_gain) * self.estimated_measurement_variance
        return self.x_hat

class ArticulationAngleFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter')
        self.measurement_buffer = [None, None]
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
        self.kalman_filter = KalmanFilterMultiMeasurement(1e-5, [1e-5, 1e-5])

    def listener_callback_markers(self, msg):
        self.measurement_buffer[0] = msg.data
        self.get_logger().info(f'Received marker measurement: {msg.data}')

    def listener_callback_range(self, msg):
        self.measurement_buffer[1] = msg.data
        self.get_logger().info(f'Received range measurement: {msg.data}')

    def process_measurements(self):
        try:
            if None not in self.measurement_buffer:
                filtered_value = self.kalman_filter.update(self.measurement_buffer)
                msg = Float64()
                msg.data = float(filtered_value)
                self.kalman_filter_publisher.publish(msg)
                self.get_logger().info(f'Filtered Articulation Angle: {filtered_value}')
                self.measurement_buffer = [None, None]
        except Exception as e:
            self.get_logger().error(f'Failed during processing: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    articulation_angle_filter = ArticulationAngleFilter()
    rclpy.spin(articulation_angle_filter)
    articulation_angle_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
