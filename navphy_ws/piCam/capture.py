import cv2
import os

# Create a folder named 'calibration_images' in the current directory
output_folder = "calibration_images"
os.makedirs(output_folder, exist_ok=True)

# Open the camera
camera = cv2.VideoCapture(0)  # 0 is the default camera index

if not camera.isOpened():
    print("Error: Could not open the camera.")
    exit()

print("Press 'c' to capture an image and save it to the folder, or 'q' to quit.")

i = 0
while True:
    # Capture frame-by-frame
    ret, frame = camera.read()

    if not ret:
        print("Error: Failed to capture image.")
        break

    # Display the live camera feed
    cv2.imshow("Camera Feed", frame)

    # Wait for a key press
    key = cv2.waitKey(1) & 0xFF  # Read key press

    if key == ord('c'):  # If 'c' is pressed, capture the frame
        # Save the captured frame as an image
        image_path = os.path.join(output_folder, f"not_calibration_{i}.png")
        cv2.imwrite(image_path, frame)
        print(f"Image saved: {image_path}")
        i += 1
    elif key == ord('q'):  # If 'q' is pressed, exit the loop
        print("Exiting...")
        break

# Release the camera and close all OpenCV windows
camera.release()
cv2.destroyAllWindows()