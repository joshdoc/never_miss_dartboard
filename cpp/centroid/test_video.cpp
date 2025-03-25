import time
import cv2
from picamera2 import Picamera2

# Initialize the camera and configure it
picam2 = Picamera2()
video_config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(video_config)

# Start the camera and begin capturing frames
picam2.start()

# Main loop to continuously capture and display frames
while True:
    frame_start = time.time()  # Start the timer for the frame

    # Capture a frame (raw data from the camera)
    image = picam2.capture_buffer("main").reshape((480, 640, 3)).astype(np.uint8)

    # Show the image
    cv2.imshow("Camera Feed", image)

    # Wait for a key press (exit the loop if 'q' is pressed)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

    # Print the time taken for processing the frame
    frame_end = time.time()
    elapsed_time = frame_end - frame_start
    print(f"Frame processed in {elapsed_time:.04f} s ({1.0/elapsed_time:.2f} FPS)")

# Clean up: stop the camera and close OpenCV windows
cv2.destroyAllWindows()
picam2.stop()
