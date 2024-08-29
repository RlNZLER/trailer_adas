#!/usr/bin/env python3

import rclpy
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import PoseArray, Pose
import image_geometry
import cv2
from cv_bridge import CvBridge, CvBridgeError
from cv2 import aruco
import numpy as np
import time 

class ArucoMarkerNode(Node):
    def __init__(self):
        super().__init__('aruco_marker_node')
        
        # Publisher for articulation angle ground truth
        self.joint_state_subscriber = self.create_subscription(JointState, 'trailer/joint_states', self.joint_state_callback, 10)
        self.ground_truth_publisher = self.create_publisher(Float64, 'articulation_angle/ground_truth', 10)
                
        # Aruco marker detection
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/depth/camera_info',self.camera_info_callback, qos_profile=qos.qos_profile_sensor_data)
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(Image, '/markers/image_with_markers', 10)
        self.marker_pose_publisher = self.create_publisher(PoseArray, '/markers/poses', 10)
        self.marker_art_angle = self.create_publisher(Float64, 'articulation_angle/markers', 10)
        
        self.time_publisher = self.create_publisher(Float64, 'articulation_angle/marker_delay', 10)
                
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict , self.parameters)
        self.camera_model = None
        
    def joint_state_callback(self, msg):
        joint_name = "trailer_joint" 
        if joint_name in msg.name:
            index = msg.name.index(joint_name)
            articulation_angle = - msg.position[index]
            
            # Publishing the ground truth articulation angle
            angle_msg = Float64()
            angle_msg.data = articulation_angle
            self.ground_truth_publisher.publish(angle_msg) # Publish the articulation angle ground truth in radians


    def camera_info_callback(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        
    def image_callback(self, msg):
        if self.camera_model is None:
            return  # Ensures the camera model is loaded
        
        start_time = time.time()  # Record the start time
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        except CvBridgeError as e:
            print(e)
            return

        corners, ids, _ = self.detector.detectMarkers(gray)     

        # Dictionary to store marker IDs and positions
        marker_positions = {}

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

                # Create pose and append to PoseArray
                pose = Pose()
                pose.position.x = tvec[0][0][0]
                pose.position.y = tvec[0][0][1]
                pose.position.z = tvec[0][0][2]
                pose_array.poses.append(pose)

                # Store the position data in the dictionary using the marker ID as the key
                marker_positions[ids[i][0]] = (pose.position.x, pose.position.y, pose.position.z)
                
            # Calculate the articulation angle after all markers have been processed
            calculated_angle = self.calculate_articulation_angle(marker_positions)
            
            end_time = time.time()  # Record the end time
            time_duration = end_time - start_time  # Calculate the time difference
            
            # Publish the time duration
            time_msg = Float64()
            time_msg.data = time_duration
            self.time_publisher.publish(time_msg)

            # Publish the PoseArray
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = 'camera_link'
            self.marker_pose_publisher.publish(pose_array)

        output_image = self.bridge.cv2_to_imgmsg(result, encoding='bgr8')
        self.publisher_.publish(output_image)
        
    def calculate_articulation_angle(self, marker_positions):
        try:
            x_values = [pos[0] for pos in marker_positions.values()]
            z_values = [pos[2] for pos in marker_positions.values()]

            if len(x_values) < 2 or len(z_values) < 2:
                return  # Not enough markers to calculate angle

            slope, _ = np.polyfit(x_values, z_values, 1)
            articulation_angle = np.arctan(slope)
            
            # Publish the marker articulation angle
            angle_msg = Float64()
            angle_msg.data = articulation_angle
            self.marker_art_angle.publish(angle_msg)

        except Exception as e:
            return None # Return None if there is an error


def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoMarkerNode()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
