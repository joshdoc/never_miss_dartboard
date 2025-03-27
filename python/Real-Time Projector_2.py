import pygame
import sys
import cv2
import numpy as np

# Initialize Pygame
pygame.init()

# Set up projector display
SCREEN_WIDTH, SCREEN_HEIGHT = 1920, 1080
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.FULLSCREEN)
pygame.display.set_caption("Dartboard Projection System")

# Load and prepare dartboard image
dartboard = cv2.imread("target_large_bullseye.png", cv2.IMREAD_UNCHANGED)

# Resize and convert color space
DARTBOARD_SIZE = 380
dartboard = cv2.resize(dartboard, (DARTBOARD_SIZE, DARTBOARD_SIZE))
dartboard = cv2.cvtColor(dartboard, cv2.COLOR_BGRA2RGBA)

# ================== Distortion Correction Setup ==================
# Define perspective transformation parameters
image_width, image_height = 640, 640
rect = np.array([[0, 0], [image_width, 0], 
                 [0, image_height], [image_width, image_height]], dtype=np.float32)

# Define trapezoidal projection points (adjust these values based on your setup)
theta = 25  # Projector tilt angle in degrees
y_shift = 15  # Vertical displacement due to tilt
trapezoid = np.array([[20, y_shift], [630, 0], 
                      [10, image_height - y_shift], [640, image_height]], dtype=np.float32)

# Compute distortion correction homography
H_distortion = cv2.getPerspectiveTransform(rect, trapezoid)

# Create warped dartboard with distortion correction
warped_dartboard = cv2.warpPerspective(dartboard, H_distortion, (image_width, image_height))

# Get warped image dimensions
warped_height, warped_width = warped_dartboard.shape[:2]

# ================== Position Prediction Setup ==================
# Homography for real-world to projector mapping (replace with your calibrated matrix)
H_position = np.array([[ 1.00658869e+01,  1.17667900e+00, 9.43843481e+02],
                       [ 8.32884912e-02, -7.26304521e+00, 5.41552715e+02],
                       [-1.69738079e-04,  1.04877030e-03,  1.00000000e+00]])

def real_to_projector(real_pos):
    """Convert real-world coordinates (cm) to projector pixels"""
    point = np.array([[[real_pos[0], real_pos[1]]]], dtype=np.float32)
    projected = cv2.perspectiveTransform(point, H_position)[0][0]
    return int(projected[0]), int(projected[1])

def draw_dartboard(center_pos):
    """Draw distortion-corrected dartboard at specified position"""
    frame = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH, 4), dtype=np.uint8)
    
    # Calculate top-left position for centering
    x = center_pos[0] - warped_width // 2
    y = center_pos[1] - warped_height // 2
    
    # Draw if within screen bounds
    if 0 <= x < SCREEN_WIDTH - warped_width and 0 <= y < SCREEN_HEIGHT - warped_height:
        # Alpha blending
        alpha = warped_dartboard[:, :, 3] / 255.0
        for c in range(3):
            frame[y:y+warped_height, x:x+warped_width, c] = \
                alpha * warped_dartboard[:, :, c] + \
                (1.0 - alpha) * frame[y:y+warped_height, x:x+warped_width, c]
    
    # Add debug info
    cv2.putText(frame, f"Projector: {center_pos}", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255, 255), 2)
    
    # Set the entire alpha channel to fully opaque
    frame[..., 3] = 255
    
    return pygame.image.frombuffer(frame.tobytes(), (SCREEN_WIDTH, SCREEN_HEIGHT), 'RGBA')

# ================== Kalman Filter Integration ==================
# Initialize position tracking; start at the origin or desired initial position
current_real = np.array([25.0, 0.0])  # Real-world coordinates in cm
current_prediction = real_to_projector(current_real)

def handle_kf_prediction(real_x_cm, real_y_cm):
    """Update position with new Kalman Filter prediction"""
    global current_prediction
    current_prediction = real_to_projector((real_x_cm, real_y_cm))

# ================== Main Loop ==================
clock = pygame.time.Clock()
running = True

while running:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            running = False

    # Replace simulation updates with direct assignment from real input:
    # For example, if you receive new coordinates (e.g., from a sensor or a Kalman filter),
    # assign them to current_real:
    # current_real = np.array([new_x, new_y])
    # For demonstration purposes, we keep the current position static.
    
    # Update projection position directly from current_real
    handle_kf_prediction(current_real[0], current_real[1])
    
    # Draw frame
    screen.fill((0, 0, 0))
    screen.blit(draw_dartboard(current_prediction), (0, 0))
    pygame.display.flip()
    clock.tick(70)

pygame.quit()
sys.exit()
