#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Float64
import numpy as np
import open3d as o3d
from sklearn.linear_model import RANSACRegressor
from sklearn.decomposition import PCA

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('point_cloud_processor')
        
        self.create_subscription(Range, 'range/u1', self.sensor_callback_u1, 10)
        self.create_subscription(Range, 'range/u2', self.sensor_callback_u2, 10)
        self.create_subscription(Range, 'range/u3', self.sensor_callback_u3, 10)
        self.create_subscription(Range, 'range/u4', self.sensor_callback_u4, 10)
        
        self.create_subscription(PointCloud2, 'camera/points', self.point_cloud_callback, 10)
        
        self.range_art_angle_pub_ = self.create_publisher(Float64, 'articulation_angle/range', 10)
        self.pc_art_angle_pub_ = self.create_publisher(Float64, 'articulation_angle/point_cloud', 10)
        self.pc_pub_ = self.create_publisher(PointCloud2, 'filtered_point_cloud', 10)
        
        self.timer_ = self.create_timer(0.1, self.process_point_cloud)
        
        self.x_u1 = -0.6125
        self.x_u2 = -0.26
        self.x_u3 = 0.26
        self.x_u4 = 0.6125
        self.points_u1 = None
        self.points_u2 = None
        self.points_u3 = None
        self.points_u4 = None
        
        self.point_cloud = None
        self.point_cloud_header = None  # Store the header for publishing

    def sensor_callback_u1(self, msg):
        self.points_u1 = (self.x_u1, msg.range)

    def sensor_callback_u2(self, msg):
        self.points_u2 = (self.x_u2, msg.range)

    def sensor_callback_u3(self, msg):
        self.points_u3 = (self.x_u3, msg.range)

    def sensor_callback_u4(self, msg):
        self.points_u4 = (self.x_u4, msg.range)
        
    def point_cloud_callback(self, msg):
        # Extract points as structured array
        points_generator = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        # Convert to list first to handle the structured nature of data
        point_list = list(points_generator)
        # Convert to numpy array using correct method to handle structured data
        if point_list:
            # Create a numpy array of shape (-1, 3) with float32 data type
            self.point_cloud = np.array(point_list).view((np.float32, 3)).reshape(-1, 3)
            self.point_cloud_header = msg.header  # Save the header for use in publishing
            self.get_logger().info(f"Received point cloud with shape: {self.point_cloud.shape}")
        else:
            self.get_logger().warn("Received empty point cloud.")

            
    def process_point_cloud(self):
        all_points = [self.points_u1, self.points_u2, self.points_u3, self.points_u4]
        filtered_points = [point for point in all_points if point is not None and point[1] <= 2]

        if len(filtered_points) < 2:
            self.get_logger().warn("Not enough points to calculate a slope.")
            return

        x_values = np.array([point[0] for point in filtered_points])
        y_values = np.array([point[1] for point in filtered_points])

        A = np.vstack([x_values, np.ones(len(x_values))]).T
        m, b = np.linalg.lstsq(A, y_values, rcond=None)[0]

        angle = -np.arctan(m)
        msg = Float64()
        msg.data = angle
        self.range_art_angle_pub_.publish(msg)

        min_range = min(y_values) - 0.05
        max_range = max(y_values) + 0.05

        if self.point_cloud is not None and self.point_cloud.ndim == 2 and self.point_cloud.shape[1] == 3:
            if not self.point_cloud_header:
                self.get_logger().error("Point cloud header is not set.")
                return

            z_filtered = self.point_cloud[(self.point_cloud[:, 2] >= min_range) & (self.point_cloud[:, 2] <= max_range)]

            if z_filtered.size > 0:
                X = z_filtered[:, :2]
                y = z_filtered[:, 2]
                ransac = RANSACRegressor()
                ransac.fit(X, y)
                inlier_mask = ransac.inlier_mask_

                pca = PCA(n_components=3)
                pca.fit(z_filtered[inlier_mask])  # Fit PCA only on inliers

                normal_vector = pca.components_[np.argmin(pca.explained_variance_)]
                angle_with_x_axis = np.arccos(normal_vector[0] / np.linalg.norm(normal_vector))

                # self.get_logger().info(f"Normal Vector: {normal_vector}")
                self.get_logger().info(f"Angle with X-axis (in radians): {angle_with_x_axis}")
                
                pc_msg = Float64()
                pc_msg.data = float(angle_with_x_axis)
                self.pc_art_angle_pub_.publish(pc_msg)

                header = self.point_cloud_header
                inlier_cloud_msg = pc2.create_cloud_xyz32(header, z_filtered[inlier_mask].tolist())
                self.pc_pub_.publish(inlier_cloud_msg)
                # self.get_logger().info("Published filtered point cloud with PCA plane analysis.")
            else:
                self.get_logger().warn("No points met the z-coordinate filtering criteria after RANSAC.")
        else:
            self.get_logger().warn("Invalid or empty point cloud data.")



def main(args=None):
    rclpy.init(args=args)
    point_cloud_node = PointCloudProcessor()
    rclpy.spin(point_cloud_node)
    point_cloud_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
