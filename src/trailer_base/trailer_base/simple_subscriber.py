# Import the rclpy library, which provides ROS 2 Python client library functions.
import rclpy
# Import the Node class from rclpy, which is used to create a ROS 2 node.
from rclpy.node import Node
# Import the String message type from the std_msgs package, which is used for receiving string messages.
from std_msgs.msg import String

# Define a class named SimpleSubscriber that inherits from Node.
class SimpleSubscriber(Node):
    
    # The constructor method of the class.
    def __init__(self):
        # Call the constructor of the parent class (Node) with the name 'simple_subscriber'.
        super().__init__('simple_subscriber')
        # Create a subscription to the 'test_topic' topic that listens for String messages.
        # The listener_callback method will be called whenever a message is received.
        self.subscription = self.create_subscription(String, 'test_topic', self.listener_callback, 10)
        # Prevent the subscription variable from being garbage collected.
        self.subscription
        
    # Define the listener callback method.
    def listener_callback(self, msg):
        # Log the received message data to the console.
        self.get_logger().info('I heard: "%s"' % msg.data)
        
# Define the main function.
def main():
    # Initialize the rclpy library.
    rclpy.init()
    # Create an instance of the SimpleSubscriber class.
    simple_subscriber = SimpleSubscriber()
    # Keep the node spinning to process callbacks.
    rclpy.spin(simple_subscriber)
    # Destroy the node explicitly (good practice).
    simple_subscriber.destroy_node()
    # Shutdown the rclpy library.
    rclpy.shutdown()
    
# Check if this script is being run directly (as opposed to being imported).
if __name__ == '__main__':
    # Call the main function.
    main()
