import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
import numpy as np

last_valid_angle = 0.0

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
        self.range_data[sensor_id] = msg.range
        self.calculate_and_publish_articulation_angle()

    def calculate_and_publish_articulation_angle(self):
        global last_valid_angle 
        
        coordinates = {
            'A': (-0.6125, self.range_data.get('u1', None)),
            'B': (-0.3125, self.range_data.get('u2', None)),
            'C': (0.3125, self.range_data.get('u3', None)),
            'D': (0.6125, self.range_data.get('u4', None))
        }
        
        filtered_points = {k: v for k, v in coordinates.items() if v[1] is not None and v[1] <= 2}
        
        if len(filtered_points) >= 2:  # Use >= 2 if two points are sufficient
            x = np.array([v[0] for v in filtered_points.values()])
            y = np.array([v[1] for v in filtered_points.values()])
            
            A = np.vstack([x, np.ones(len(x))]).T
            angle_rad = last_valid_angle  # Default to last_valid_angle in case of exception
            
            try:
                m, c = np.linalg.lstsq(A, y, rcond=None)[0]
                angle_rad = np.arctan(m)  # Update angle based on current regression
                last_valid_angle = angle_rad
            except np.linalg.LinAlgError:
                self.get_logger().warn("Linear algebra error in fitting model, using last valid angle")
                
        else:
            angle_rad = last_valid_angle
        
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
