import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class FilterData(Node):
    def __init__(self):
        super().__init__('filter_data')
        self.u1_sub_ = self.create_subscription(Range, 'range/u1', self.u1_filter_callback, 10)
        self.u1_pub_ = self.create_publisher(Range, 'range/u1_filtered', 10)
        
    def u1_filter_callback(self, msg):
        if msg.range < 2.0:
            self.u1_pub_.publish(msg)
        else:
            self.get_logger().info("Filtered out unrealistic range value: %s", msg.range)
            
def main(args=None):
    rclpy.init(args=args)
    filter_data = FilterData()
    rclpy.spin(filter_data)
    filter_data.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()