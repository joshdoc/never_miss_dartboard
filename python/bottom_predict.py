import time
import numpy as np
from filterpy.kalman import KalmanFilter
import matplotlib.pyplot as plt

# ----------------------------
# Camera and Model Parameters
# ----------------------------
# Camera intrinsics (from calibration)
f = 800           # focal length in pixels
cx, cy = 320, 240 # principal point in pixels
Y_const = 1.83    # fixed distance from camera (in meters) at which the dart flies

# Process model parameters
k_v = 0.67  # drag coefficient (per second)
g = 9.81    # gravity is not used here since height is assumed constant

# ----------------------------
# Helper Functions
# ----------------------------
def pixel_to_world(u, v):
    """
    Convert undistorted pixel coordinates (u,v) to world coordinates (x,y)
    using the pinhole model and a fixed Y_const.
    """
    # Here, we assume that the world coordinates (x,y) are in the horizontal plane.
    x = (u - cx) * Y_const / f
    y = (v - cy) * Y_const / f
    return np.array([x, y])

def dynamics_F(dt):
    """
    Build the state transition matrix F for the linear system.
    We use Euler integration with drag for the velocities:
       x_new = x + v_x*dt
       v_x_new = v_x - k_v*v_x*dt
    (Similarly for y.)
    """
    F = np.array([[1, 0, dt,  0],
                  [0, 1,  0, dt],
                  [0, 0, 1 - k_v*dt, 0],
                  [0, 0, 0, 1 - k_v*dt]])
    return F

def dynamics_Q(dt, sigma_process=0.1):
    """
    Process noise covariance matrix Q.
    Here we use a simple model that scales with dt.
    """
    Q = sigma_process**2 * np.array([[dt**4/4,      0, dt**3/2,      0],
                                       [     0, dt**4/4,      0, dt**3/2],
                                       [dt**3/2,      0, dt**2,        0],
                                       [     0, dt**3/2,      0, dt**2]])
    return Q

# ----------------------------
# Simulation of Real-Time Measurements
# ----------------------------
def get_uart_data():
    """
    Placeholder function to simulate receiving data via UART.
    Returns:
      measurement: undistorted pixel centroid [u, v]
      delta_t: time since last measurement in seconds
    In practice, this function will read from the UART.
    """
    # For simulation, we create a synthetic measurement.
    # Let's assume the dart follows a straight-line trajectory in world space:
    # true state: x(t) = x0 + v_x * t, y(t) = y0 + v_y * t, with constant velocities.
    # We'll simulate some noise in the pixel measurements.
    global true_state_sim, last_time_sim
    current_time = time.time()
    delta_t = current_time - last_time_sim
    last_time_sim = current_time
    
    # Propagate the "true" state (for simulation only)
    # Use the same discrete dynamics as our model
    F = dynamics_F(delta_t)
    true_state_sim = F @ true_state_sim
    # Convert true world state (x, y) to pixel measurements using the pinhole model
    u = cx + (f * true_state_sim[0]) / Y_const
    v = cy + (f * true_state_sim[1]) / Y_const
    
    # Add measurement noise (simulate undistorted pixel measurement)
    noise = np.random.normal(0, 1, size=2)  # 1-pixel noise std dev
    measurement = np.array([u, v]) + noise
    return measurement, delta_t

# ----------------------------
# Initialize the Kalman Filter
# ----------------------------
kf = KalmanFilter(dim_x=4, dim_z=2)
# Initial state: assume the dart starts at the origin with an initial velocity.
# For example, initial speed 7 m/s at 5 degrees relative to x-axis.
initial_speed = 7
initial_angle = np.radians(5)
initial_state = np.array([0, 0,
                          initial_speed * np.cos(initial_angle),
                          initial_speed * np.sin(initial_angle)])
kf.x = initial_state

# Initial covariance
kf.P = np.eye(4) * 0.1

# Measurement matrix: we directly measure x and y (after pixel -> world conversion)
kf.H = np.array([[1, 0, 0, 0],
                 [0, 1, 0, 0]])

# Measurement noise covariance (assumed, in world coordinates, e.g., ~0.05 m std dev)
kf.R = np.eye(2) * 0.05**2

# We will update F and Q at each time step based on delta_t.
# Process noise (Q) parameters can be tuned.

# ----------------------------
# Real-Time Processing Loop
# ----------------------------
# For simulation, we initialize a true state for the dart (for generating synthetic measurements)
true_state_sim = initial_state.copy()
last_time_sim = time.time()

start_time = time.time()
time_limit = 0.175  # 175 milliseconds
timestamps = []
kf_states = []

while (time.time() - start_time) < time_limit:
    measurement_pixels, delta_t = get_uart_data()  # measurement in pixel coordinates and dt in seconds
    # Convert pixel measurement to world coordinates
    measurement_world = pixel_to_world(*measurement_pixels)
    
    # Update the Kalman filter with the proper F and Q for the current delta_t
    F = dynamics_F(delta_t)
    kf.F = F
    kf.Q = dynamics_Q(delta_t, sigma_process=0.1)
    
    kf.predict()
    kf.update(measurement_world)
    
    timestamps.append(time.time() - start_time)
    kf_states.append(kf.x.copy())
    
    print(f"t = {(time.time()-start_time)*1000:.1f} ms, KF state: {kf.x}")

# ----------------------------
# Extrapolate to x = 2.5 m
# ----------------------------
# Get the final UKF state at the end of the updates.
final_state = kf.x.copy()  # [x, y, vx, vy]
x_current, y_current, vx_current, vy_current = final_state

target_x = 2.5
if vx_current <= 0:
    print("Warning: vx is non-positive; cannot extrapolate to target x.")
    predicted_y = y_current
else:
    # For our linear (or near-linear) model with drag,
    # we can compute the time to reach target_x approximately.
    # Here we use a simple approximation that neglects drag in the prediction interval:
    dt_to_target = (target_x - x_current) / vx_current
    # More accurate: incorporate drag if needed (for simplicity, we use linear extrapolation)
    predicted_y = y_current + vy_current * dt_to_target

print(f"\nPredicted left/right position (y) at x = {target_x} m: {predicted_y:.3f} m")

# ----------------------------
# Plotting (optional)
# ----------------------------
kf_states = np.array(kf_states)
plt.figure(figsize=(8, 6))
plt.plot(kf_states[:, 0], kf_states[:, 1], 'b.-', label='KF Estimate')
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("Kalman Filter Estimated Trajectory")
plt.legend()
plt.grid()
plt.show()
