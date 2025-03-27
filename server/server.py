# ===================================================================
# Section: Imports
# ===================================================================

import sys
import select
import serial
import time

import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

import pygame
import cv2

# ===================================================================
# Section: UART Functions
# ===================================================================

def configure_port(port, baudrate=115200, timeout=1):
    try:
        # Open the port in read-write mode for bidirectional communication
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        return ser
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}")
        return None

def send_start_command(ser_list):
    start_cmd = "START\n".encode('ascii')
    for ser in ser_list:
        try:
            ser.write(start_cmd)
            print(f"Sent start command on {ser.port}")
        except Exception as e:
            print(f"Error sending start command on {ser.port}: {e}")

# ===================================================================
# Section: Pygame Projection Initialization
# ===================================================================

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

# ===================================================================
# Section: Filtering Initialization
# ===================================================================

# Camera parameters (from calibration)
cx, cy = 320, 240   # Principal point (image center in pixels)
f = 800             # Focal length in pixels
y_const = 1.83      # Fixed real-world z height (used in projection)

# Define sigma points for UKF
points = MerweScaledSigmaPoints(n=5, alpha=0.01, beta=2, kappa=0)

# Initialize the UKF
ukf = UKF(dim_x=5, dim_z=2, fx=fx, hx=hx, dt=0.01, points=points)
ukf.x = np.array([0, 0, 8 * np.cos(np.radians(5)), 8 * np.sin(np.radians(5))])  # initial state: [x, z, vx, vz]
ukf.P *= 0.3   # initial covariance
ukf.R *= 6     # measurement noise covariance (in pixel space)
ukf.Q *= 0.2   # process noise covariance
ukf.Q[4,4] = 0.01

x_final = 2.5  # The x position where we predict z
latest_dt = 0.01  # Default small dt value in case it's missing

# ===================================================================
# Section: UART Initialization
# ===================================================================

# Opens & Configures UART Ports
port0 = "/dev/ttyAMA0"  # Camera 1
port2 = "/dev/ttyAMA2"  # Camera 2
ser0 = configure_port(port0)
ser2 = configure_port(port2)

msg_cnt = 0

# Error-Handling
if not ser0 or not ser2:
    print("Failed to open one or more UART ports.")
    sys.exit(1)

# Wait for user input to send start command
ports = [ser0, ser2]
print("Press ENTER to send start capture command to cameras...")
rlist, _, _ = select.select([sys.stdin], [], [])
if sys.stdin in rlist:
    line = sys.stdin.readline()
    if line.strip().lower() == "start" or line.strip() == "":
        send_start_command(ports)

# ===================================================================
# Section: Prediction Functions
# ===================================================================

# Define the state transition function (2D kinematics with kv drag)
def fx(state, dt):
    """
    State transition function for 2D projectile motion with kv drag.
    State: [x, z, vx, vz]
    """
    x, z, vx, vz, Y = state
    
    kv = 0.67  # Linear drag coefficient
    g = 9.81   # Gravity (acts in -z direction)
    
    # Compute acceleration
    ax = -kv * vx
    az = -kv * vz - g  # Include gravity
    
    # Update velocity (Euler integration)
    vx_new = vx + ax * dt
    vz_new = vz + az * dt
    
    # Update position
    x_new = x + vx * dt
    z_new = z + vz * dt
    Y_new = Y
    return np.array([x_new, z_new, vx_new, vz_new, Y_new])

# Measurement function: maps (x, z) to pixel coordinates (u, v)
def hx(state):
    x, z, vx, vz, Y = state
    u = cx + (f * x) / Y
    v = cy + (f * z) / Y
    return np.array([u, v])

# Analytical propagation for x(t) and z(t)
def propagate_state_analytical(state, t, kv=0.67, g=9.81):
    """
    Propagate the state analytically over time t.
    For x-dynamics:
        x(t) = x0 + (vx0/kv) * (1 - exp(-kv*t))
        vx(t) = vx0 * exp(-kv*t)
    For z-dynamics:
        vz(t) = (vz0 + g/kv)*exp(-kv*t) - g/kv
        z(t) = z0 + (vz0 + g/kv)/kv * (1 - exp(-kv*t)) - g*t/kv
    """
    x0, z0, vx0, vz0 = state
    x = x0 + (vx0 / kv) * (1 - np.exp(-kv*t))
    vx = vx0 * np.exp(-kv*t)
    vz = (vz0 + g/kv) * np.exp(-kv*t) - g/kv
    z = z0 + ((vz0 + g/kv)/kv) * (1 - np.exp(-kv*t)) - (g*t)/kv
    return np.array([x, z, vx, vz])

def time_to_target(x0, vx0, target, kv):
    """
    Compute the time required for the x component to reach 'target' given:
    x(t) = x0 + (vx0/kv)*(1 - exp(-kv*t))
    Solving:
        t = -1/kv * ln(1 - (kv*(target - x0))/vx0)
    """
    arg = 1 - (kv * (target - x0)) / vx0
    if arg <= 0:
        return None
    return -np.log(arg) / kv

# ===================================================================
# Section: Projection Functions
# ===================================================================

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

# ===================================================================
# Section: Main
# ===================================================================

def main():

# Main Loop

    while True:

        rlist, _, _ = select.select([ser0.fileno(), ser2.fileno()], [], [])
        for fd in rlist:
            if fd == ser0.fileno():
                data = ser0.read(6)
                if len(data) == 6:
                    t = int.from_bytes(data[0:2], byteorder='little')
                    c1 = int.from_bytes(data[2:4], byteorder='little')
                    c2 = int.from_bytes(data[4:6], byteorder='little')
                    print(f"Received on {ser0.port} (Client 1): time={t} ms, centroid1={c1}, centroid2={c2}")

                    if msg_cnt == 0:
                        start_time = time.time()  # Record start time

                    pixel_measurement, latest_dt = (c1, c2), t / 1000

                    # **UKF Prediction and Update**
                    ukf.predict(dt=latest_dt)  
                    ukf.update(pixel_measurement)

                    print(f"Time: {elapsed_time:.2f} ms, Estimated State: {ukf.x}")

                    msg_cnt += 1

            # if fd == ser2.fileno():
            #     data = ser2.read(6)
            #     if len(data) == 6:
            #         t = int.from_bytes(data[0:2], byteorder='little')
            #         c1 = int.from_bytes(data[2:4], byteorder='little')
            #         c2 = int.from_bytes(data[4:6], byteorder='little')
            #         print(f"Received on {ser2.port} (Client 2): time={t} ms, centroid1={c1}, centroid2={c2}")

        elapsed_time = (time.time() - start_time) * 1000  # Convert to ms
        if elapsed_time >= 175:
            break

        # event_happened = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (
                event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False
                break

        # # Check for new position update
        # new_position = get_new_dartboard_position()
        # if new_position is not None:
        #     # Update the current real-world position and projection
        #     current_real = np.array(new_position)
        #     handle_kf_prediction(current_real[0], current_real[1])
        #     event_happened = True

        # # Only update the projection if there's a new event (or if you want to force redraw)
        # if event_happened:
        #     screen.fill((0, 0, 0))
        #     screen.blit(draw_dartboard(current_prediction), (0, 0))
        #     pygame.display.flip()

    z_final = propagate_state_analytical(ukf.x, x_final)
    print(f"Predicted Height at x={x_final}m: {z_final:.3f}m")

    handle_kf_prediction(0, z_final)

if __name__ == "__main__":
    main()

pygame.quit()
sys.exit()