import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
import matplotlib.pyplot as plt
import time
# Camera parameters (from calibration)
cx, cy = 320, 240   # Principal point (image center in pixels)
f = 800             # Focal length in pixels
y_const = 1.83      # Fixed real-world z height (used in projection)

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

# Define sigma points for UKF
points = MerweScaledSigmaPoints(n=5, alpha=0.01, beta=2, kappa=0)

# Initialize the UKF
ukf = UKF(dim_x=5, dim_z=2, fx=fx, hx=hx, dt=0.01, points=points)
ukf.x = np.array([0, 0, 8 * np.cos(np.radians(5)), 8 * np.sin(np.radians(5))])  # initial state: [x, z, vx, vz]
ukf.P *= 0.3   # initial covariance
ukf.R *= 6     # measurement noise covariance (in pixel space)
ukf.Q *= 0.2   # process noise covariance
ukf.Q[4,4] = 0.01
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


# **Real-Time Loop**
start_time = time.time()  # Record start time
x_final = 2.5  # The x position where we predict z
latest_dt = 0.01  # Default small dt value in case it's missing

while True:
    # Check if 175ms has passed
    elapsed_time = (time.time() - start_time) * 1000  # Convert to ms
    if elapsed_time >= 175:
        break

    # **Receive measurement via UART (Assume this part is handled elsewhere)**
    ## function to get the next data from "black"/side camera
    pixel_measurement, latest_dt = None #get_uart_data()  # Function must return (u, v) and delta_t

    # **UKF Prediction and Update**
    ukf.predict(dt=latest_dt)  
    ukf.update(pixel_measurement)

    print(f"Time: {elapsed_time:.2f} ms, Estimated State: {ukf.x}")

# **After 175ms, Predict z at x = 2.5m using analytical propagation**
z_final = propagate_state_analytical(ukf.x, x_final)

print(f"Predicted Height at x={x_final}m: {z_final:.3f}m")