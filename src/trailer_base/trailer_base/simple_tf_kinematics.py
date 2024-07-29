import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class SimpleTfKinematics(Node):
    
    def __init__(self):
        super().__init__('simple_tf_kinematics')
        
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        self.static_transform_stamped_ = TransformStamped()
        self.static_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_stamped_.header.frame_id = 'base_footprint'
        self.static_transform_stamped_.child_frame_id = 'base_link'
        
        self.static_transform_stamped_.transform.translation.x = 0.0
        self.static_transform_stamped_.transform.translation.y = 0.0
        self.static_transform_stamped_.transform.translation.z = 0.28575
        
        self.static_transform_stamped_.transform.rotation.x = 0.0
        self.static_transform_stamped_.transform.rotation.y = 0.0
        self.static_transform_stamped_.transform.rotation.z = 0.0
        self.static_transform_stamped_.transform.rotation.w = 1.0
        
        self.static_tf_broadcaster.sendTransform(self.static_transform_stamped_)
        
        self.get_logger().info('Publishing static transform from %s to %s' % 
                                (self.static_transform_stamped_.header.frame_id, self.static_transform_stamped_.child_frame_id))
        
def main():
    rclpy.init()
    simple_tf_kinematics = SimpleTfKinematics()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()