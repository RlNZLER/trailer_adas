#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

class MultiRadarPointCloud(Node):
    def __init__(self):
        super().__init__('multi_radar_point_cloud')

        # Create subscriptions for all four radar topics
        self.create_subscription(PointCloud, 'range/u1', self.point_cloud_callback_u1, 10)
        self.create_subscription(PointCloud, 'range/u2', self.point_cloud_callback_u2, 10)
        self.create_subscription(PointCloud, 'range/u3', self.point_cloud_callback_u3, 10)
        self.create_subscription(PointCloud, 'range/u4', self.point_cloud_callback_u4, 10)
        
        # 
        self.pc_art_angle_pub_ = self.create_publisher(Float64, 'articulation_angle/point_cloud', 10)
        
        # Initialize the points
        self.y_u1 = -0.6125
        self.y_u2 = -0.26
        self.y_u3 = 0.26
        self.y_u4 = 0.6125
        self.points_u1 = (0, self.y_u1)
        self.points_u2 = (0, self.y_u2)
        self.points_u3 = (0, self.y_u3)
        self.points_u4 = (0, self.y_u4)
        
        # Initialize Matplotlib figure and 3D axis
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Initialize scatter plots for each radar with larger point size
        self.scatter_u1 = self.ax.scatter([], [], [], s=20, c='r', label='Radar U1')
        self.scatter_u2 = self.ax.scatter([], [], [], s=20, c='g', label='Radar U2')
        self.scatter_u3 = self.ax.scatter([], [], [], s=20, c='b', label='Radar U3')
        self.scatter_u4 = self.ax.scatter([], [], [], s=20, c='y', label='Radar U4')
        
        # Setup plot limits for a smaller scale
        self.ax.set_xlim(0, 2)
        self.ax.set_ylim(0, 2)
        self.ax.set_zlim(0, 2)

        # Set labels and legend
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.legend()

        # To store points from each radar
        self.points_u1 = []
        self.points_u2 = []
        self.points_u3 = []
        self.points_u4 = []

        # Call the animation function repeatedly
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)
        
    def point_cloud_callback_u1(self, msg):
        x_values_all = np.array([point.x for point in msg.points])
        if x_values_all.size > 0:
            # Set a threshold to filter x-values, e.g., mean + standard deviation
            threshold = np.mean(x_values_all) + np.std(x_values_all)
            x_values = x_values_all[x_values_all > threshold]
            mean_x = np.mean(x_values) if x_values.size > 0 else 0
        else:
            mean_x = 0
        self.points_u1 = (mean_x, self.y_u1, 0)

    def point_cloud_callback_u2(self, msg):
        x_values_all = np.array([point.x for point in msg.points])
        if x_values_all.size > 0:
            # Set a threshold to filter x-values, e.g., mean + standard deviation
            threshold = np.mean(x_values_all) + np.std(x_values_all)
            x_values = x_values_all[x_values_all > threshold]
            mean_x = np.mean(x_values) if x_values.size > 0 else 0
        else:
            mean_x = 0
        self.points_u2 = (mean_x, self.y_u2, 0)

    def point_cloud_callback_u3(self, msg):
        x_values_all = np.array([point.x for point in msg.points])
        if x_values_all.size > 0:
            # Set a threshold to filter x-values, e.g., mean + standard deviation
            threshold = np.mean(x_values_all) + np.std(x_values_all)
            x_values = x_values_all[x_values_all > threshold]
            mean_x = np.mean(x_values) if x_values.size > 0 else 0
        else:
            mean_x = 0
        self.points_u3 = (mean_x, self.y_u3, 0)

    def point_cloud_callback_u4(self, msg):
        x_values_all = np.array([point.x for point in msg.points])
        if x_values_all.size > 0:
            # Set a threshold to filter x-values, e.g., mean + standard deviation
            threshold = np.mean(x_values_all) + np.std(x_values_all)
            x_values = x_values_all[x_values_all > threshold]
            mean_x = np.mean(x_values) if x_values.size > 0 else 0
        else:
            mean_x = 0
        self.points_u4 = (mean_x, self.y_u4, 0)
        
    def update_plot(self, frame):
        self.ax.clear()
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(-1, 1)
        # Corrected method to unpack x, y, z for scatter
        self.ax.scatter(self.points_u1[0], self.points_u1[1], self.points_u1[2], c='r', label='Radar U1')
        self.ax.scatter(self.points_u2[0], self.points_u2[1], self.points_u2[2], c='g', label='Radar U2')
        self.ax.scatter(self.points_u3[0], self.points_u3[1], self.points_u3[2], c='b', label='Radar U3')
        self.ax.scatter(self.points_u4[0], self.points_u4[1], self.points_u4[2], c='y', label='Radar U4')
        self.ax.legend()
        plt.draw()
        
def main(args=None):
    rclpy.init(args=args)
    node = MultiRadarPointCloud()
    
    # Use a separate thread to run the ROS2 spin loop
    import threading
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Start the Matplotlib event loop
    plt.show()
    
    # Clean up after plot window is closed
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()