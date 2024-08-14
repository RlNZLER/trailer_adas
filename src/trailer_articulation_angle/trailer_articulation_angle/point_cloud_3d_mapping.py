#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class MultiRadarPointCloud(Node):
    def __init__(self):
        super().__init__('multi_radar_point_cloud')

        # Create subscriptions for all four radar topics
        self.create_subscription(PointCloud, 'range/u1', self.point_cloud_callback_u1, 10)
        self.create_subscription(PointCloud, 'range/u2', self.point_cloud_callback_u2, 10)
        self.create_subscription(PointCloud, 'range/u3', self.point_cloud_callback_u3, 10)
        self.create_subscription(PointCloud, 'range/u4', self.point_cloud_callback_u4, 10)
        
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
        self.points_u1 = np.array([(point.x, point.y, point.z) for point in msg.points])

    def point_cloud_callback_u2(self, msg):
        self.points_u2 = np.array([(point.x, point.y, point.z) for point in msg.points])

    def point_cloud_callback_u3(self, msg):
        self.points_u3 = np.array([(point.x, point.y, point.z) for point in msg.points])

    def point_cloud_callback_u4(self, msg):
        self.points_u4 = np.array([(point.x, point.y, point.z) for point in msg.points])

    def update_plot(self, frame):
        if len(self.points_u1) > 0:
            self.scatter_u1._offsets3d = (self.points_u1[:, 0], self.points_u1[:, 1], self.points_u1[:, 2])
        if len(self.points_u2) > 0:
            self.scatter_u2._offsets3d = (self.points_u2[:, 0], self.points_u2[:, 1], self.points_u2[:, 2])
        if len(self.points_u3) > 0:
            self.scatter_u3._offsets3d = (self.points_u3[:, 0], self.points_u3[:, 1], self.points_u3[:, 2])
        if len(self.points_u4) > 0:
            self.scatter_u4._offsets3d = (self.points_u4[:, 0], self.points_u4[:, 1], self.points_u4[:, 2])
        return self.scatter_u1, self.scatter_u2, self.scatter_u3, self.scatter_u4

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
