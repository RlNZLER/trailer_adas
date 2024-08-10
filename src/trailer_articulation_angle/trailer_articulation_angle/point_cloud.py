import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
import numpy as np

class PointCloud(Node):
    def __init__(self):
        super().__init__('point_cloud')
        sensor_ids = ['u1', 'u2', 'u3', 'u4']
        for sensor_id in sensor_ids:
            setattr(self, f'{sensor_id}_sub_', self.create_subscription(Range, f'range/{sensor_id}', lambda msg, sid=sensor_id: self.sensor_callback(msg, sid), 10))
            
        self.pc_art_angle_pub_ = self.create_publisher(Float64, 'articulation_angle/point_cloud', 10)
        
        self.range_data = {
            "u1": None,
            "u2": None,
            "u3": None,
            "u4": None
        }
        
    def sensor_callback(self, msg, sensor_id):
        if msg.range <= 2.0:  # Process only valid range data
            self.range_data[sensor_id] = msg.range
            self.calculate_and_publish_articulation_angle()

    def calculate_and_publish_articulation_angle(self):
        coordinates = {
            'A': (-0.6125, self.range_data.get('u1', None)),
            'B': (-0.26, self.range_data.get('u2', None)),
            'C': (0.26, self.range_data.get('u3', None)),
            'D': (0.6125, self.range_data.get('u4', None))
        }
        
        # Filter out points with range greater than 2 meters
        filtered_points = {k: v for k, v in coordinates.items() if v[1] is not None and v[1] <= 2}
        
        # Check if we have at least two points to perform regression
        if len(filtered_points) < 2:
            self.get_logger().info('Not enough data points to calculate angle.')
            return  # No sufficient data to calculate angle
        
        # Prepare data for regression
        x = np.array([v[0] for v in filtered_points.values()])
        y = np.array([v[1] for v in filtered_points.values()])
        
        # Perform linear regression
        A = np.vstack([x, np.ones(len(x))]).T
        
        try:
            m, c = np.linalg.lstsq(A, y, rcond=None)[0]
        except np.linalg.LinAlgError:
            self.get_logger().error('Linear regression failed due to singular matrix.')
            return  # Linear regression failed

        # Calculate the angle in radians and then convert to degrees
        angle_rad = np.arctan(m)
        
        # Publish the angle
        angle_msg = Float64()
        angle_msg.data = angle_rad
        self.pc_art_angle_pub_.publish(angle_msg)
            

def main(args=None):
    rclpy.init(args=args)
    filter_data = PointCloud()
    rclpy.spin(filter_data)
    filter_data.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
