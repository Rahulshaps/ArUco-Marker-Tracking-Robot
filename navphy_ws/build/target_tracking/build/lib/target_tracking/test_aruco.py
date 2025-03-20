import cv2
import os
import numpy as np
# import matplotlib.pyplot as plt
import time

# Create a folder for saving captured images
output_folder = "captured_images"
os.makedirs(output_folder, exist_ok=True)

# Load camera parameters
camera_matrix = np.array([[615.0, 0.0, 320.0], [0.0, 615.0, 240.0], [0.0, 0.0, 1.0]])
dist_coeffs = np.zeros((4, 1))  # Assuming no distortion
marker_size = 79  # Marker size in mm
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
aruco_params = cv2.aruco.DetectorParameters()

# Open the camera
camera = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not camera.isOpened():
    print("Error: Could not open the camera.")
    exit()

print("Press 'c' to capture an image, or 'q' to quit.")

# List to store measured distances
measured_distances = []
expected_distances = [200, 300, 400, 500]  # Expected distances in mm
timestamps = []  # This will store time (or fake time)
start_time = time.time()
i = 0
while True:
    # Capture frame-by-frame
    ret, frame = camera.read()

    if not ret or frame is None:
        print("Error: Failed to capture image.")
        continue

    # Detect ArUco markers
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is not None:
        # Draw the detected markers
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Estimate pose of each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)

        if len(rvecs) == len(ids) and len(tvecs) == len(ids):
            for rvec, tvec, corner, marker_id in zip(rvecs, tvecs, corners, ids.flatten()):
                distance = np.linalg.norm(tvec)
                print(f"Marker ID {marker_id}: Distance = {distance:.2f} mm")

                # Store the measured distance
                measured_distances.append(distance)
                # Record a timestamp (using fake time or real time)
                current_time = time.time() - start_time  # Fake time: time since start
                timestamps.append(current_time)
                
                # Show the distance in the image
                text = f"Distance: {distance:.2f} mm"
                # Use the top-left corner of the marker for placing the text
                top_left_corner = tuple(corner[0][0].astype(int))  # First corner in each marker
                cv2.putText(frame, text, top_left_corner, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                # Draw the marker outline
                cv2.polylines(frame, [np.int32(corner)], isClosed=True, color=(0, 255, 0), thickness=2)

    # Show the live feed with markers and distance
    cv2.imshow('ArUco Marker Detection', frame)

    # Wait for user input to capture or quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord('c'):
        # Save the captured frame as an image
        image_path = os.path.join(output_folder, f"calibration_{i}.png")
        cv2.imwrite(image_path, frame)
        print(f"Image saved: {image_path}")
        i += 1
    elif key == ord('q'):
        print("Exiting...")
        break

# Release the camera and clean up
camera.release()
cv2.destroyAllWindows()

# Now plot the measured distances vs fake time
plt.figure(figsize=(10, 6))

# Plot expected distances as horizontal lines
for expected_distance in expected_distances:
    plt.plot(timestamps, [expected_distance] * len(timestamps), 'r--', label=f"Expected: {expected_distance} mm")

# Plot measured distances
plt.plot(timestamps, measured_distances, 'bo-', label="Measured Distance")

plt.title("Measured vs Expected Distance Over Time")
plt.xlabel("Time (seconds)")
plt.ylabel("Distance (mm)")
plt.legend()
plt.grid(True)
plt.show()
