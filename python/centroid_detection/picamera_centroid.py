import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

# -----------------------
# Tuning parameters
# -----------------------
RESOLUTION = (1280, 720)     # 720p resolution
FPS = 90                     # Target FPS (actual max FPS depends on your camera settings)
THRESHOLD_VALUE = 127        # Binary threshold value (0-255)
MAX_THRESHOLD = 255          # Maximum value for thresholding
MIN_CONTOUR_AREA = 500       # Minimum contour area to filter noise

# -----------------------
# Camera initialization
# -----------------------
camera = PiCamera()
camera.resolution = RESOLUTION
camera.framerate = FPS
rawCapture = PiRGBArray(camera, size=RESOLUTION)

# Allow the camera to warm up
time.sleep(0.1)

# -----------------------
# Video capture loop
# -----------------------
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array

    # Convert to grayscale for thresholding
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply binary thresholding
    ret, thresh = cv2.threshold(gray, THRESHOLD_VALUE, MAX_THRESHOLD, cv2.THRESH_BINARY)
    
    # Find contours in the thresholded image
    contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Option: Process the largest contour (assumed to be the tracked object)
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > MIN_CONTOUR_AREA:
            # Calculate centroid using image moments
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Draw the centroid on the image
                cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(image, "Centroid", (cx - 25, cy - 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    # Display the result
    cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF

    # Clear the stream for the next frame
    rawCapture.truncate(0)

    # Exit on pressing 'q'
    if key == ord("q"):
        break

cv2.destroyAllWindows()
