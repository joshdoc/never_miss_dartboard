import time
import cv2
import numpy as np
from picamera2 import Picamera2

# Initialize camera with proper 720p70 configuration
picam2 = Picamera2()
video_config = picam2.create_video_configuration(
    main={"size": (1280, 720), "format": "RGB888"},
    controls={"FrameRate": 70}  # Correct framerate setting
)
picam2.configure(video_config)
picam2.start()

while True:
    frame_start = time.time()
    
    # Capture frame and convert to BGR
    image_rgb = picam2.capture_buffer("main").reshape((720, 1280, 3)).astype(np.uint8)
    image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    
    cv2.imshow("720p70 Centroid Tracking", image_bgr)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    print(f"Processing: {time.time() - frame_start:.3f}s | FPS: {1/(time.time() - frame_start):.1f}")

cv2.destroyAllWindows()
picam2.stop()
