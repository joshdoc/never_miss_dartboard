import time
import cv2
import numpy as np
from picamera2 import Picamera2
import sys
import select

# Initialize camera
picam2 = Picamera2()
video_config = picam2.create_video_configuration(
    main={"size": (1280, 720), "format": "RGB888"},
    controls={"FrameRate": 70}
)
picam2.configure(video_config)
picam2.start()

# Set up non-blocking input
def has_input():
    return select.select([sys.stdin], [], [], 0)[0]

print("Headless capture running. Press 'q' then Enter to quit...")

try:
    while True:
        frame_start = time.time()
        
        # Capture frame (no display conversion needed)
        image_rgb = picam2.capture_buffer("main").reshape((720, 1280, 3)).astype(np.uint8)
        
        # Check for quit signal
        if has_input():
            key = sys.stdin.read(1)
            if key == 'q':
                break

        # Optional: Keep FPS measurement
        elapsed = time.time() - frame_start
        print(f"Frame time: {elapsed:.3f}s | FPS: {1/elapsed:.1f}", end='\r')

finally:
    picam2.stop()
    print("\nCamera stopped. Exiting cleanly.")
