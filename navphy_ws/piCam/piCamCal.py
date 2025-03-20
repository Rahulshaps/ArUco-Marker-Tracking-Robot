#!/usr/bin/env python
import numpy as np
import cv2
import os
from datetime import datetime
import scipy.io as sio


# Set up the ArUco dictionary and parameters
markerType = cv2.aruco.DICT_5X5_50  # ( Choose from: "DICT_4X4_50", "DICT_5X5_50", "DICT_6X6_50" )
markerLength = 25 # mm

aruco_dict = cv2.aruco.getPredefinedDictionary(markerType)
aruco_params = cv2.aruco.DetectorParameters()
#detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)


########## OPEN CAMERA ##########
camera = cv2.VideoCapture(1)
camera.set(3, 1000)
camera.set(4, 750)
if not camera.isOpened():
    print("[ERROR] Could not open camera.")
    exit()
print("[INFO] Camera is open. Press 'esc' to close.")

i = 0 # To keep track of saved images
k = 0

###### LOOPS UNTIL Q IS PRESSED ######
while True:

    success, frame = camera.read()  # Captures an image from the camera.
    if not success or frame is None:
        print("[ERROR] Could not read frame from camera.")
        break
    try:
        s = frame.shape # Check frame shape to avoid errors
    except AttributeError as e:
        print(f"[ERROR] Could not retrieve frame shape: {e}")
        break


    # Take a pic of frame for calibration!
    if cv2.waitKey(33) == ord('c'):
            print("Taking pic {}...".format(i))
            cv2.imwrite("piCalibration/image_{}.png".format(k), frame)
            k += 1


# Release resources
camera.release()
cv2.destroyAllWindows()
print("[INFO] Video stream stopped.")

