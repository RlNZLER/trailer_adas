import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import math

class PointCloud(Node):
    def __init__(self):
        super().__init__('point_cloud')
        self.subscription = self.create_subscription(
                Range,
                '/range/u1',  # Replace with your topic name
                self.listener_callback,
                10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        try:
            # Validate that the data is a proper float and within expected ranges
            if not isinstance(msg.range, float):
                raise ValueError("Range is not a float.")
            
            # Your existing checks and logging
            if math.isnan(msg.range):
                self.get_logger().warn("Received NaN in range value")
                return
            if math.isinf(msg.range):
                self.get_logger().warn("Received Inf in range value")
                return

            # Continue processing
            if msg.range < msg.min_range or msg.range > msg.max_range:
                self.get_logger().warn(f"Range value out of bounds: {msg.range}")
            else:
                self.get_logger().info(f"Range value within bounds: {msg.range}")

        except Exception as e:
            # Handle and log any error, then continue
            self.get_logger().error(f"Error processing message: {str(e)}")
            self.get_logger().debug(f"Problematic message: {msg}")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloud()
    try:
        rclpy.spin(node)
    except RuntimeError as e:
        # Log the error and keep the node running
        node.get_logger().error(f"Caught runtime error: {str(e)}")
    except Exception as e:
        # Handle any other exceptions
        node.get_logger().error(f"Caught exception: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
