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
DARTBOARD_SIZE = 380
dartboard = cv2.resize(dartboard, (DARTBOARD_SIZE, DARTBOARD_SIZE))
dartboard = cv2.cvtColor(dartboard, cv2.COLOR_BGRA2RGBA)

# ================== Distortion Correction Setup ==================
image_width, image_height = 640, 640
rect = np.array([[0, 0], [image_width, 0],
                 [0, image_height], [image_width, image_height]], dtype=np.float32)
theta = 25  # Projector tilt angle in degrees
y_shift = 15  # Vertical displacement due to tilt
trapezoid = np.array([[20, y_shift], [630, 0],
                      [10, image_height - y_shift], [640, image_height]], dtype=np.float32)
H_distortion = cv2.getPerspectiveTransform(rect, trapezoid)
warped_dartboard = cv2.warpPerspective(dartboard, H_distortion, (image_width, image_height))
warped_height, warped_width = warped_dartboard.shape[:2]

# ================== Position Prediction Setup ==================
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
        alpha = warped_dartboard[:, :, 3] / 255.0
        for c in range(3):
            frame[y:y+warped_height, x:x+warped_width, c] = (
                alpha * warped_dartboard[:, :, c] +
                (1.0 - alpha) * frame[y:y+warped_height, x:x+warped_width, c]
            )
    
    # Add debug info
    cv2.putText(frame, f"Projector: {center_pos}", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255, 255), 2)
    
    # Set alpha channel fully opaque
    frame[..., 3] = 255
    return pygame.image.frombuffer(frame.tobytes(), (SCREEN_WIDTH, SCREEN_HEIGHT), 'RGBA')

# ================== Kalman Filter Integration ==================
current_real = np.array([25.0, 0.0])  # Initial real-world position (cm)
current_prediction = real_to_projector(current_real)

def handle_kf_prediction(real_x_cm, real_y_cm):
    """Update position with new Kalman Filter prediction"""
    global current_prediction
    current_prediction = real_to_projector((real_x_cm, real_y_cm))

def get_new_dartboard_position():
    """
    This function should be replaced with your method for obtaining new
    dartboard positions (e.g., from sensors or a Kalman filter).
    Here, it returns None if there is no new data.
    """
    # Example: Uncomment and implement actual data acquisition
    # new_x, new_y = sensor.read_position()  # Replace with real sensor data
    # return (new_x, new_y)
    return None  # No new position by default

# ================== Main Event Loop ==================
running = True
while running:
    event_happened = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (
            event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            running = False
            break

    # Check for new position update
    new_position = get_new_dartboard_position()
    if new_position is not None:
        # Update the current real-world position and projection
        current_real = np.array(new_position)
        handle_kf_prediction(current_real[0], current_real[1])
        event_happened = True

    # Only update the projection if there's a new event (or if you want to force redraw)
    if event_happened:
        screen.fill((0, 0, 0))
        screen.blit(draw_dartboard(current_prediction), (0, 0))
        pygame.display.flip()

    # Here you might include a slight delay to prevent CPU overuse
    pygame.time.wait(10)

pygame.quit()
sys.exit()
