# This script is an entry point to the Aruco marker detection and pose estimation.
# It uses the camera calibration values to estimate the pose of the markers.
# The script will display the original image and the image with the detected markers and their pose.
# It is using the OpenCV library 4.10+ which has the latest Aruco functions

import cv2
import cv2.aruco as aruco
import numpy as np
import time # We will use this to ensure a steady processing rate


# Load the camera calibration values
camera_calibration = np.load('Calibration.npz')
CM=camera_calibration['CM'] #camera matrix
dist_coef=camera_calibration['dist_coef']# distortion coefficients from the camera

# Define the ArUco dictionary and parameters
marker_size = 0.03  # Size of the marker in meters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Define a processing rate
processing_period = 0.25

# Create two OpenCV named windows
cv2.namedWindow("Frame", cv2.WINDOW_AUTOSIZE)
cv2.namedWindow("Gray", cv2.WINDOW_AUTOSIZE)

# Position the windows next to each other
cv2.moveWindow("Gray", 640, 100)
cv2.moveWindow("Frame", 0, 100)
# Start capturing video
cap = cv2.VideoCapture(1)

# Set the starting time
start_time = time.time()
fps = 0

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # --- Insert crop here: keep narrow central vertical strip ---
    h, w = frame.shape[:2]
    narrow_ratio = 0.1   # keep 10% of width (adjust as needed)
    new_w = int(w * narrow_ratio)
    x1 = (w - new_w) // 2
    x2 = x1 + new_w
    frame = frame[:, x1:x2]  # replace frame with the cropped view
    # --- end insert ---

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('gray-image', gray)

    # Detect markers
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # If markers are detected
    if ids is not None:
        # Draw detected markers
        frame = aruco.drawDetectedMarkers(frame, corners, ids)

        # Estimate pose of each marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, CM, dist_coef)

        for rvec, tvec in zip(rvecs, tvecs):
            # Draw axis for each marker
            t = tvec[0]
            distance = np.linalg.norm(t)
            forward_z = t[2]
            frame = cv2.drawFrameAxes(frame, CM, dist_coef, rvec, tvec, 0.05)
             # Put distance text near marker (you can adjust position)
            c = corners[0][0].astype(int) if len(corners) else None
            text_pos = (10, 90)
            cv2.putText(frame, f"Dist: {distance:.2f} m (z={forward_z:.2f} m)", text_pos,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
    # Add the frame rate to the image
    cv2.putText(frame, f"CAMERA FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(frame, f"PROCESSING FPS: {1/processing_period:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Display the resulting frame
    cv2.imshow('Frame', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Ensure a steady processing rate
    elapsed_time = time.time() - start_time
    fps = 1 / elapsed_time
    if elapsed_time < processing_period:
        time.sleep(processing_period - elapsed_time)
    start_time = time.time()

# When everything is done, release the capture and close windows
cap.release()
cv2.destroyAllWindows()