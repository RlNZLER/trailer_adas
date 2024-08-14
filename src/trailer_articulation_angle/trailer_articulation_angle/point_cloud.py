#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
import numpy as np

class PointCloud(Node):
    def __init__(self):
        super().__init__('point_cloud')
        
        self.create_subscription(Range, 'range/u1', self.point_cloud_callback_u1, 10)
        self.create_subscription(Range, 'range/u2', self.point_cloud_callback_u2, 10)
        self.create_subscription(Range, 'range/u3', self.point_cloud_callback_u3, 10)
        self.create_subscription(Range, 'range/u4', self.point_cloud_callback_u4, 10)
        
        # Publish the articulation angle in radians
        self.pc_art_angle_pub_ = self.create_publisher(Float64, 'articulation_angle/point_cloud', 10)
        self.timer_ = self.create_timer(0.1, self.calculate_articulation_angle)
        
        self.x_u1 = -0.6125
        self.x_u2 = -0.26
        self.x_u3 = 0.26
        self.x_u4 = 0.6125
        self.points_u1 = None
        self.points_u2 = None
        self.points_u3 = None
        self.points_u4 = None

    def point_cloud_callback_u1(self, msg):
        self.points_u1 = (self.x_u1, msg.range)

    def point_cloud_callback_u2(self, msg):
        self.points_u2 = (self.x_u2, msg.range)

    def point_cloud_callback_u3(self, msg):
        self.points_u3 = (self.x_u3, msg.range)

    def point_cloud_callback_u4(self, msg):
        self.points_u4 = (self.x_u4, msg.range)

    def calculate_articulation_angle(self):
        all_points = [self.points_u1, self.points_u2, self.points_u3, self.points_u4]
        filtered_points = [point for point in all_points if point is not None and point[1] <= 2]

        if len(filtered_points) < 2:
            self.get_logger().warn("Not enough points to calculate a slope.")
            return
        
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
        self.pc_art_angle_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    point_cloud_node = PointCloud()
    rclpy.spin(point_cloud_node)
    point_cloud_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
