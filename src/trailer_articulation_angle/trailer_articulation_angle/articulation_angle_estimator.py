import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, PointCloud2
import numpy as np
import cv2
import open3d as o3d
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2

class ArticulationAngleNode(Node):
    def __init__(self):
        super().__init__('articulation_angle_node')  # Node name set here

        # Subscriptions to the ultrasonic sensors
        self.sub_u1 = self.create_subscription(Range, 'range/u1', self.ultrasonic_callback, 10)
        self.sub_u2 = self.create_subscription(Range, 'range/u2', self.ultrasonic_callback, 10)
        self.sub_u3 = self.create_subscription(Range, 'range/u3', self.ultrasonic_callback, 10)
        self.sub_u4 = self.create_subscription(Range, 'range/u4', self.ultrasonic_callback, 10)

        # Subscription to the depth camera point cloud
        self.sub_depth_camera = self.create_subscription(PointCloud2, 'depth_camera/point_cloud', self.depth_camera_callback, 10)

        # Initialize sensor data
        self.ultrasonic_data = { 'u1': None, 'u2': None, 'u3': None, 'u4': None }
        self.point_cloud = None

        self.bridge = CvBridge()

    def ultrasonic_callback(self, msg):
        self.ultrasonic_data[msg.header.frame_id] = msg.range

    def depth_camera_callback(self, msg):
        self.point_cloud = self.convert_pointcloud2_to_xyz(msg)

    def convert_pointcloud2_to_xyz(self, ros_point_cloud):
        # Convert ROS PointCloud2 to Open3D PointCloud
        pc = o3d.geometry.PointCloud()
        # Extract points from PointCloud2 message
        points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(ros_point_cloud, field_names=("x", "y", "z"), skip_nans=True)])
        pc.points = o3d.utility.Vector3dVector(points)
        return pc

    def calculate_articulation_angle(self):
        # Ensure all data is available
        if None in self.ultrasonic_data.values() or self.point_cloud is None:
            return

        # Use ultrasonic data
        # Example: u1 and u2 for angle calculation
        angle_ultrasonic = self.calculate_angle_from_ultrasonic(self.ultrasonic_data['u1'], self.ultrasonic_data['u2'])

        # Use Hough Transform on depth camera data
        angle_depth_camera = self.calculate_angle_from_depth_camera(self.point_cloud)

        self.get_logger().info(f'Ultrasonic Angle: {angle_ultrasonic}, Depth Camera Angle: {angle_depth_camera}')

    def calculate_angle_from_ultrasonic(self, distance1, distance2):
        # Implement your logic to calculate angle using distance1 and distance2
        # Placeholder example:
        baseline = 1.0  # Replace with the actual distance between the sensors
        angle = np.arctan2(distance2 - distance1, baseline)
        return np.degrees(angle)

    def calculate_angle_from_depth_camera(self, point_cloud):
        # Convert point cloud to 2D image for Hough Transform
        height, width = 480, 640  # Assuming a resolution for depth camera
        image = np.zeros((height, width), dtype=np.uint8)

        # Populate the image with point cloud data
        for point in np.asarray(point_cloud.points):
            x, y, z = point
            # Convert 3D point to 2D pixel (simple projection, adjust as needed)
            u = int((x / z) * width / 2 + width / 2)
            v = int((y / z) * height / 2 + height / 2)
            if 0 <= u < width and 0 <= v < height:
                image[v, u] = 255

        # Apply Hough Transform
        edges = cv2.Canny(image, 50, 150, apertureSize=3)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)
        
        if lines is not None:
            for rho, theta in lines[0]:
                angle = np.degrees(theta)
                return angle
        return None

    def timer_callback(self):
        self.calculate_articulation_angle()

def main(args=None):
    rclpy.init(args=args)
    node = ArticulationAngleNode()
    timer_period = 0.1  # seconds
    node.create_timer(timer_period, node.timer_callback)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
