# Import the rclpy library, which provides ROS 2 Python client library functions.
import rclpy
# Import the Node class from rclpy, which is used to create a ROS 2 node.
from rclpy.node import Node
# Import the String message type from the std_msgs package, which is used for publishing string messages.
from std_msgs.msg import String

# Define a class named SimplePublisher that inherits from Node.
class SimplePublisher(Node):
    
        # The constructor method of the class.
        def __init__(self):
            # Call the constructor of the parent class (Node) with the name 'simple_publisher'.
            super().__init__('simple_publisher')
            # Create a publisher that will publish String messages on the 'topic' topic with a queue size of 10.
            self.publisher_ = self.create_publisher(String, 'test_topic', 10)
            # Define the timer period (0.5 seconds).
            timer_period = 0.5  # seconds
            # Create a timer that will call the timer_callback method every timer_period seconds.
            self.timer = self.create_timer(timer_period, self.timer_callback)
    
        # Define the timer callback method.
        def timer_callback(self):
            # Create a new String message.
            msg = String()
            # Set the data of the message to 'Hello, World!'.
            msg.data = 'Hello, World!'
            # Publish the message.
            self.publisher_.publish(msg)
            # Log the message data to the console.
            self.get_logger().info('Publishing: "%s"' % msg.data)
            
# Define the main function.
def main():
    # Initialize the rclpy library.
    rclpy.init()
    # Create an instance of the SimplePublisher class.
    simple_publisher = SimplePublisher()
    # Keep the node spinning to process callbacks.
    rclpy.spin(simple_publisher)
    # Destroy the node explicitly (good practice).
    simple_publisher.destroy_node()
    # Shutdown the rclpy library.
    rclpy.shutdown()

# Check if this script is being run directly (as opposed to being imported).
if __name__ == '__main__':
    # Call the main function.
    main()
