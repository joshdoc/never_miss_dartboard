import cv2
import numpy as np
import matplotlib.pyplot as plt

# Parameters
VIDEO_PATH = "video_capture/throw_2_trimmed.mp4"  # Change to your video file path
OUTPUT_VIDEO_PATH = "video_capture/throw_2_centroid.mp4"  # Output file name
DOWNSCALE_FACTOR = 1  # Adjust resolution scale (1 = original, <1 = downscale)
THRESHOLD_MIN = 100  # Min threshold for binarization
THRESHOLD_MAX = 255  # Max threshold for binarization
BOX_FILTER_SIZE = (5, 5)  # Kernel size for box filter

# Open video capture
cap = cv2.VideoCapture(VIDEO_PATH)
if not cap.isOpened():
    print("Error: Cannot open video file.")
    exit()

# Get original video properties
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) * DOWNSCALE_FACTOR)
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) * DOWNSCALE_FACTOR)
fps = int(cap.get(cv2.CAP_PROP_FPS))

# Define the video writer (MP4 format)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for .mp4 format
out = cv2.VideoWriter(OUTPUT_VIDEO_PATH, fourcc, fps, (frame_width, frame_height))

frame_count = 0
centroids_x = []  # Store x-coordinates
centroids_y = []  # Store y-coordinates

print("frame_number,cx,cy")  # Print CSV-like header

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break  # Stop if video ends

    # Resize frame
    frame = cv2.resize(frame, (frame_width, frame_height), interpolation=cv2.INTER_AREA)

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply box filter
    filtered = cv2.blur(gray, BOX_FILTER_SIZE)

    # Apply binary thresholding
    _, binary = cv2.threshold(filtered, THRESHOLD_MIN, THRESHOLD_MAX, cv2.THRESH_BINARY)

    # Convert binary image back to 3-channel grayscale (for saving in color video format)
    binary_color = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Select the largest contour (if any exist)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)  # Get the largest contour

        # Compute centroid
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:  # Avoid division by zero
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(binary_color, (cx, cy), 5, (0, 0, 255), -1)  # Draw only the largest centroid

            # Store centroid coordinates
            centroids_x.append(cx)
            centroids_y.append(cy)

            # Print centroid (CSV format)
            print(f"{frame_count + 1},{cx},{cy}")

    # Write processed frame to output video
    out.write(binary_color)

    # Increment frame count
    frame_count += 1

# Release resources
cap.release()
out.release()

# Perform quadratic curve fitting
if len(centroids_x) >= 3:  # Need at least 3 points for quadratic fit
    coefficients = np.polyfit(centroids_x, centroids_y, 2)  # Fit y = ax² + bx + c
    a, b, c = coefficients  # Extract coefficients

    # Compute predicted y-values using the quadratic equation
    predicted_y = [a * x**2 + b * x + c for x in centroids_x]

    # Calculate R-squared value
    y_mean = np.mean(centroids_y)  # Mean of actual y-values
    ss_total = sum((y - y_mean) ** 2 for y in centroids_y)  # Total variance
    ss_residual = sum((y - y_pred) ** 2 for y, y_pred in zip(centroids_y, predicted_y))  # Residual variance
    r_squared = 1 - (ss_residual / ss_total)  # Compute R²

    print("\nQuadratic Fit Equation:")
    print(f"y = {a:.6f}x² + {b:.6f}x + {c:.6f}")
    print(f"\nGoodness of Fit (R²): {r_squared:.6f}")

    # Plot the fitted curve
    plt.figure(figsize=(8, 6))
    plt.scatter(centroids_x, centroids_y, color='blue', label='Detected Centroids')  # Plot detected points
    x_fit = np.linspace(min(centroids_x), max(centroids_x), 100)  # Generate smooth x values
    y_fit = a * x_fit**2 + b * x_fit + c  # Compute y values from the fit
    plt.plot(x_fit, y_fit, color='red', linestyle='--', label='Fitted Quadratic Curve')  # Plot quadratic fit

    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.title("Dart Trajectory with Quadratic Fit")
    plt.legend()
    plt.gca().invert_yaxis()  # Flip y-axis for image coordinate system
    plt.show()

else:
    print("\nNot enough data points for quadratic fitting.")

print(f"\nProcessing complete. Video saved as: {OUTPUT_VIDEO_PATH}")
