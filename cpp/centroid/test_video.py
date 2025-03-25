import cv2
import numpy as np
import time

def process_frame(frame, kernel, threshold_value, scale_factor):
    """
    Process a single frame: downsample, apply top-hat filtering,
    thresholding, contour detection, and centroid marking.
    
    Returns:
        processed_frame: The downsampled frame with the centroid (if found) drawn.
        binary: The binary image after thresholding.
        centroid: (x, y) coordinates of the detected centroid or None.
    """
    # Downsample the image
    frame_ds = cv2.resize(frame, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_AREA)
    
    # Convert to grayscale and apply Gaussian blur
    gray = cv2.cvtColor(frame_ds, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Apply morphological top-hat filtering
    top_hat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, kernel)
    
    # Threshold the top-hat result to isolate bright regions
    _, binary = cv2.threshold(top_hat, threshold_value, 255, cv2.THRESH_BINARY)
    
    # Find contours from the binary image
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centroid = None
    if contours:
        # Choose the largest contour (assumed to be the feature of interest)
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centroid = (cx, cy)
            # Draw a red circle on the downsampled frame at the centroid
            cv2.circle(frame_ds, (cx, cy), 5, (0, 0, 255), -1)
    return frame_ds, binary, centroid

def main():
    # Option 1: If your camera is exposed via V4L2, you can use:
    cap = cv2.VideoCapture(0)
    
    # Option 2: If you need to use a GStreamer pipeline with libcamera-vid, uncomment and adjust the pipeline:
    # pipeline = ("libcamerasrc ! video/x-raw, width=1280, height=720, framerate=70/1 ! "
    #             "videoconvert ! appsink")
    # cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    
    # Set desired resolution and FPS (note: support depends on your camera and OS)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 70)
    
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return

    # Parameters (adjust kernel size, threshold, and scale factor as needed)
    kernel_size = (15, 15)
    threshold_value = 30
    scale_factor = 0.5  # Downsampling factor
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, kernel_size)

    print("Starting live video processing. Press 'q' to quit.")
    
    total_time = 0
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        start_time = time.time()
        
        # Process the frame
        processed_frame, binary_frame, centroid = process_frame(frame, kernel, threshold_value, scale_factor)
        
        processing_time = time.time() - start_time
        total_time += processing_time
        frame_count += 1
        print(f"Frame {frame_count} processed in {processing_time:.4f} seconds, Centroid: {centroid}")
        
        # Display the processed images
        cv2.imshow("Processed Frame", processed_frame)
        cv2.imshow("Binary Top-Hat", binary_frame)
        
        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    avg_time = total_time / frame_count if frame_count > 0 else 0
    print(f"Average processing time per frame: {avg_time:.4f} seconds")
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

