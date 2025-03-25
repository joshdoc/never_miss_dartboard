import time
import cv2
import numpy as np
from picamera2 import Picamera2

# Initialize the camera with 720p resolution at 70 FPS
picam2 = Picamera2()
video_config = picam2.create_video_configuration(
    main={"size": (1280, 720), "format": "RGB888"},
    sensor={"output_size": (1280, 720), "framerate": 70}
)
picam2.configure(video_config)
picam2.start()

while True:
    frame_start = time.time()
    
    # Capture and convert frame to BGR format
    image_rgb = picam2.capture_buffer("main").reshape((720, 1280, 3)).astype(np.uint8)
    image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    
    # Convert to grayscale and threshold
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    # Calculate centroid using image moments
    M = cv2.moments(thresh)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX, cY = 640, 360  # Center coordinates if no object detected
    
    # Draw centroid marker
    cv2.circle(image_bgr, (cX, cY), 7, (0, 0, 255), -1)
    
    # Display frame
    cv2.imshow("Centroid Tracking", image_bgr)
    
    # Exit on 'q' press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    # Performance metrics
    print(f"Frame Time: {time.time() - frame_start:.4f}s | FPS: {1/(time.time() - frame_start):.1f}")

# Cleanup
cv2.destroyAllWindows()
picam2.stop()
