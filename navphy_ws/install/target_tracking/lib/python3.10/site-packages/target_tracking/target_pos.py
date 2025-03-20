#!/usr/bin/env python
import numpy as np
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
import cv2
import scipy.io as sio
from cv2 import aruco
import os

class ArucoPublisher(Node):
  def __init__(self):
    super().__init__('target_pos')

    #### Initialize publisher
    self.publisher = self.create_publisher(Float32MultiArray, '/aruco_pose', 10)
    self.targetspottedpub = self.create_publisher(Int32, '/target_spotted', 10)

    self.spotted = Int32()

    #### Camera parameters
    # self.camParams = sio.loadmat(os.path.join(os.getcwd(), "aruco/aruco/asus_camParams.mat"))
    # self.cameraMatrix = self.camParams['cameraMatrix']
    # self.distCoeffs = self.camParams['distortionCoefficients']
    self.cameraMatrix = np.array([[615.0, 0.0, 320.0], [0.0, 615.0, 240.0], [0.0, 0.0, 1.0]])
    self.distCoeffs = np.zeros((4, 1))  # Assuming no distortion

    #### ArUco marker parameters
    self.marker_size = 79  # Default marker size in c=mm 

    # Initialize ArUco detector with the specified type
    self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    self.aruco_params = cv2.aruco.DetectorParameters()

    self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not self.camera.isOpened():
      self.get_logger().error("Could not open camera!")
      exit()
    else:
      self.get_logger().info("Camera successfully opened.")
    
    #### Initialize timer for periodic detection and publishing
    self.timer = self.create_timer(0.1, self.detect_and_publish)  # 10 Hz, adjust as needed

    # Set up other variables
    self.marker_position = Float32MultiArray()
    self.marker_position.data = [0.0, 0.0, 0.0]  # Initialize with zeros (rvec, tvec)

  def detect_and_publish(self):
    #"""Capture frame, detect ArUco markers, and publish their pose."""
    ret, frame = self.camera.read()
    
    self.spotted.data = 0

    if ret:
      # Detect ArUco markers
      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
      if ids is not None and len(ids) > 0:
        for i in range(len(ids)):
          if ids[i] == 1:
            # Estimate pose of each marker
            _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_size, self.cameraMatrix, self.distCoeffs)
            
            if tvecs is not None and len(corners[i]) > 0:
              for tvec in tvecs:
                # Flatten the translation vector (tvec), convert to meters if necessary
                self.marker_position.data = list((tvec.flatten()) / 1000.0)  # Convert to meters
                self.publisher.publish(self.marker_position)
                self.spotted.data = 1
      
    self.targetspottedpub.publish(self.spotted)

  def cleanup(self):
    """Release camera resources and destroy any OpenCV windows."""
    self.camera.release()
    cv2.destroyAllWindows()
  
def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the ArUcoPublisher node
    aruco_publisher = ArucoPublisher()
    
    # Spin the node so the callback function is called
    rclpy.spin(aruco_publisher)

    # Clean up resources when the node is destroyed
    aruco_publisher.cleanup()

    # Destroy the node explicitly
    aruco_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
  main()