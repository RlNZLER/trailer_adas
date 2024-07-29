import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from geometry_msgs.msg import TransformStamped
from trailer_msgs.srv import GetTransform
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class SimpleTfKinematics(Node):
    
    def __init__(self):
        super().__init__('simple_tf_kinematics')
        
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.dynameic_tf_broadcaster = TransformBroadcaster(self)
        
        self.static_transform_stamped_ = TransformStamped()
        self.dynamic_transform_stamped_ = TransformStamped()
        
        self.x_increment_ = 0.05
        self.last_x_ = 0.0
        
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        
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
        
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        
        self.get_transform_srv_ = self.create_service(GetTransform, 'get_transform', self.get_transform_callback)
        
    def timer_callback(self):
        self.dynamic_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_stamped_.header.frame_id = 'odom'
        self.dynamic_transform_stamped_.child_frame_id = 'base_footprint'
        
        self.dynamic_transform_stamped_.transform.translation.x = self.last_x_ + self.x_increment_
        self.dynamic_transform_stamped_.transform.translation.y = 0.0
        self.dynamic_transform_stamped_.transform.translation.z = 0.0
        
        self.dynamic_transform_stamped_.transform.rotation.x = 0.0
        self.dynamic_transform_stamped_.transform.rotation.y = 0.0
        self.dynamic_transform_stamped_.transform.rotation.z = 0.0
        self.dynamic_transform_stamped_.transform.rotation.w = 1.0
        
        self.dynameic_tf_broadcaster.sendTransform(self.dynamic_transform_stamped_)
        
        self.last_x_ = self.dynamic_transform_stamped_.transform.translation.x
        
    def get_transform_callback(self, request, response):
        self.get_logger().info('Requested transform between %s to %s' % (request.frame_id, request.child_frame_id))
        requested_transform = TransformStamped()
        try:
            requested_transform = self.tf_buffer_.lookup_transform(request.frame_id, request.child_frame_id, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error('Failed to get transform: %s' % e)
            response.success = False
            return response
        
        response.transform = requested_transform
        response.success = True
        return response
    
    
def main():
    rclpy.init()
    simple_tf_kinematics = SimpleTfKinematics()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()