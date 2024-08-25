import rclpy
from rclpy.node import Node
import pandas as pd
from std_msgs.msg import Float64
import signal
import sys
import threading
import time

class ArticulationAngleLogger(Node):
    def __init__(self):
        super().__init__('articulation_angle_logger')
        
        # Initialize subscriptions
        self.create_subscription(Float64, '/articulation_angle/ground_truth_timestamp', self.ground_truth_timestamp_callback, 10)
        self.create_subscription(Float64, '/articulation_angle/ground_truth', self.ground_truth_callback, 10)
        self.create_subscription(Float64, '/articulation_angle/markers_timestamp', self.markers_timestamp_callback, 10)
        self.create_subscription(Float64, '/articulation_angle/markers', self.markers_callback, 10)
        self.create_subscription(Float64, '/articulation_angle/range_timestamp', self.range_timestamp_callback, 10)
        self.create_subscription(Float64, '/articulation_angle/range', self.range_callback, 10)
        self.create_subscription(Float64, '/articulation_angle/kalman_timestamp', self.kalman_timestamp_callback, 10)
        self.create_subscription(Float64, '/articulation_angle/filtered', self.filtered_callback, 10)
        
        # Data storage dictionary
        self.data = {
            'timestamp': [],
            'ground_truth_timestamp': [],
            'ground_truth': [],
            'markers_timestamp': [],
            'markers': [],
            'range_timestamp': [],
            'range': [],
            'kalman_timestamp': [],
            'filtered': []
        }
        
        # CSV output file
        self.output_file = 'articulation_angle_log.csv'

        # Register signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGHUP, self.signal_handler)  # Handle terminal close

        # Start the periodic save timer
        self.save_interval = 10  # Save data every 10 seconds
        self.timer_thread = threading.Thread(target=self.periodic_save)
        self.timer_thread.start()

    def record_data(self, topic, value):
        # Append timestamp for each data entry
        timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

        self.data['timestamp'].append(timestamp)

        # Add the value to the appropriate list in the dictionary
        for key in self.data.keys():
            if key == topic:
                self.data[key].append(value)
            elif key == 'timestamp':
                continue
            else:
                # Ensure all lists are of equal length by appending None to missing data
                if len(self.data[key]) < len(self.data['timestamp']):
                    self.data[key].append(None)

    def filtered_callback(self, msg):
        self.record_data('filtered', msg.data)

    def ground_truth_callback(self, msg):
        self.record_data('ground_truth', msg.data)

    def ground_truth_timestamp_callback(self, msg):
        self.record_data('ground_truth_timestamp', msg.data)

    def kalman_timestamp_callback(self, msg):
        self.record_data('kalman_timestamp', msg.data)

    def markers_callback(self, msg):
        self.record_data('markers', msg.data)

    def markers_timestamp_callback(self, msg):
        self.record_data('markers_timestamp', msg.data)

    def range_callback(self, msg):
        self.record_data('range', msg.data)

    def range_timestamp_callback(self, msg):
        self.record_data('range_timestamp', msg.data)

    def save_to_csv(self):
        # Convert the dictionary to a pandas DataFrame
        df = pd.DataFrame(self.data)
        df.to_csv(self.output_file, index=False)
        self.get_logger().info(f'Data saved to {self.output_file}')

    def periodic_save(self):
        while rclpy.ok():
            self.save_to_csv()
            time.sleep(self.save_interval)

    def shutdown(self):
        self.save_to_csv()  # Save one last time on shutdown
        self.destroy_node()
        rclpy.shutdown()

    def signal_handler(self, signum, frame):
        self.get_logger().info(f'Received signal {signum}, shutting down...')
        self.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = ArticulationAngleLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Logging interrupted by user')
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()