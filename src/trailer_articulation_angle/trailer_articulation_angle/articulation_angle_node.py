import rclpy
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
import image_geometry
import cv2
from cv_bridge import CvBridge, CvBridgeError
from cv2 import aruco
import numpy as np

'''
/depth_camera/camera_info
/depth_camera/depth/camera_info
/depth_camera/depth/image_raw
/depth_camera/depth/image_raw/compressed
/depth_camera/depth/image_raw/compressedDepth
/depth_camera/depth/image_raw/theora
/depth_camera/image_raw
/depth_camera/image_raw/compressed
/depth_camera/image_raw/compressedDepth
/depth_camera/image_raw/theora
/depth_camera/points

'''


class ArticulationAngleNode(Node):
    def __init__(self):
        super().__init__('articulation_angle_node')
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_camera/depth/camera_info',self.camera_info_callback, qos_profile=qos.qos_profile_sensor_data)
        self.subscription = self.create_subscription(Image, '/depth_camera/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(Image, '/visualization/image_with_markers', 10)
        self.pose_publisher = self.create_publisher(PoseArray, '/markers/poses', 10)
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict , self.parameters)
        self.camera_model = None

    def camera_info_callback(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        
    def image_callback(self, msg):
        if self.camera_model is None:
            return  # Ensures the camera model is loaded

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        except CvBridgeError as e:
            print(e)
            return

        corners, ids, _ = self.detector.detectMarkers(gray)     

        if ids is not None and len(ids) > 0:
            pose_array = PoseArray()
            # Limit to at most 5 markers
            if len(ids) > 5:
                ids = ids[:5]
                corners = corners[:5]

            # Draw all detected markers
            aruco.drawDetectedMarkers(frame, corners, ids)

            for i, corner in enumerate(corners):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, 0.05, self.camera_model.intrinsicMatrix(), self.camera_model.distortionCoeffs())
                result = cv2.drawFrameAxes(frame, self.camera_model.intrinsicMatrix(), self.camera_model.distortionCoeffs(), rvec, tvec, 0.1)

                pose = Pose()
                pose.position.x = tvec[0][0][0]
                pose.position.y = tvec[0][0][1]
                pose.position.z = tvec[0][0][2]
                pose_array.poses.append(pose)

            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = 'depth_camera_link'
            self.pose_publisher.publish(pose_array)

        output_image = self.bridge.cv2_to_imgmsg(result, encoding='bgr8')
        self.publisher_.publish(output_image)


def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArticulationAngleNode()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
