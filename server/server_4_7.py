import socket
import struct
import time
from datetime import datetime
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints

################## WiFi #########################

DEST_PORT = 4600  # Port must match sender

def setup_socket(port=DEST_PORT):
    """Set up and return a blocking UDP socket bound to the given port."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', port))
    sock.setblocking(False)  # Non-blocking for integration into main loop
    return sock

def receive_centroid(sock):
    """
    Attempts to receive a 5-byte UDP packet.
    Returns:
        (pi_id, cx, cy, timestamp) if valid
        None if no data or invalid format
    """
    try:
        data, addr = sock.recvfrom(1024)
        if len(data) == 5:
            pi_id = data[0]
            cx, cy = struct.unpack('!HH', data[1:5])
            timestamp = datetime.now().isoformat(timespec='microseconds')
            return pi_id, cx, cy, timestamp
        else:
            print(f"Received invalid packet length: {len(data)} from {addr}")
            return None
    except BlockingIOError:
        return None  # No data available this cycle


############################# UKF and PREDICTION ###################################

# ------------------------------------------------------------
# Calibration Parameters
# ------------------------------------------------------------
# Floor camera (Pi 2) intrinsics:
intrinsics_left = np.load("intrinsic_white.npz")
K_left = intrinsics_left["camera_matrix"]   
dist_left = intrinsics_left["dist_coeffs"]

# Side camera (Pi 1) intrinsics:
intrinsics_right = np.load("intrinsic_black.npz")
K_right = intrinsics_right["camera_matrix"]
dist_right = intrinsics_right["dist_coeffs"]

# Load extrinsics 
extrinsics = np.load('stereo.npz')
R_side = extrinsics['R']      # rotation
T_side = extrinsics['T']      # translation

# ------------------------------------------------------------
# Measurement Functions
# ------------------------------------------------------------
def hx_floor(state):
    """
    Measurement function for the floor camera (Pi 2).
    Assumes the floor camera's coordinate system is the world frame.
    Projects a 3D state into 2D pixel coordinates.
    """
    p = state[:3]
    X, Y, Z = p
    if Z < 1e-3:
        Z = 1e-3
    u = K_left[0, 0] * (X / Z) + K_left[0, 2]
    v = K_left[1, 1] * (Y / Z) + K_left[1, 2]
    return np.array([u, v])

def hx_side(state):
    """
    Measurement function for the side camera (Pi 1).
    Transforms the world point into the side camera frame using R_side and T_side,
    then projects using the pinhole model with K_right.
    """
    p = state[:3]
    p_cam = R_side @ p + T_side.reshape(3)
    X, Y, Z = p_cam
    if Z < 1e-3:
        Z = 1e-3
    u = K_right[0, 0] * (X / Z) + K_right[0, 2]
    v = K_right[1, 1] * (Y / Z) + K_right[1, 2]
    return np.array([u, v])

# Measurement noise covariance
R_meas = np.array([[16.0, 0],
                   [0,    9.0]])

# ------------------------------------------------------------
# Dynamics
# ------------------------------------------------------------
g = 9.81   # gravitational acceleration
kv = 0.50   # drag coefficient 

def fx(state, dt):
    """
    Process model: projectile dynamics with gravity and drag.
    State: [x, y, z, vx, vy, vz]
    Uses Euler integration.
    """
    x, y, z, vx, vy, vz = state
    ax = -kv * vx
    ay = -kv * vy
    az = -kv * vz - g
    return np.array([x + vx * dt,
                     y + vy * dt,
                     z + vz * dt,
                     vx + ax * dt,
                     vy + ay * dt,
                     vz + az * dt])

# ------------------------------------------------------------
# UKF Setup
# ------------------------------------------------------------
dim_x = 6  # state dimension: [x, y, z, vx, vy, vz]
dim_z = 2  # measurement dimension: [u, v]
points = MerweScaledSigmaPoints(n=dim_x, alpha=0.1, beta=2.0, kappa=0)
# Initialize UKF using a default measurement function (we will switch based on camera)
ukf = UnscentedKalmanFilter(dim_x=dim_x, dim_z=dim_z, dt=0.02, fx=fx, hx=hx_floor, points=points)
# Set an initial state guess (modify as needed)
ukf.x = np.array([0.8, 0.0, 1.8, -9.5, 0.0, 0.0])
ukf.P = np.eye(dim_x) * 1.0
ukf.Q = np.eye(dim_x) * 0.5
ukf.Q[0][0] = 0.025
ukf.Q[1][1] = 0.05
ukf.Q[2][2] = 0.025
ukf.Q[3][3] = 1.0
ukf.Q[4][4] = 0.1
ukf.Q[5][5] = 0.5
# ------------------------------------------------------------
# State Propagation
# ------------------------------------------------------------

def propagate_state_analytical(state, t, kv=0.67, g=9.81):
    """
    Analytically propagate a 6D state over time t.
    
    State is assumed to be [x, y, z, vx, vy, vz].
    
    For x and y dynamics (without gravity):
      x(t) = x0 + (vx0/kv)*(1 - exp(-kv*t))
      vx(t) = vx0 * exp(-kv*t)
      y(t) = y0 + (vy0/kv)*(1 - exp(-kv*t))
      vy(t) = vy0 * exp(-kv*t)
      
    For z dynamics (with gravity):
      z(t) = z0 + ((vz0 + g/kv)/kv)*(1 - exp(-kv*t)) - (g*t)/kv
      vz(t) = (vz0 + g/kv)*exp(-kv*t) - g/kv
    """
    x0, y0, z0, vx0, vy0, vz0 = state
    x = x0 + (vx0/kv) * (1 - np.exp(-kv*t))
    vx = vx0 * np.exp(-kv*t)
    y = y0 + (vy0/kv) * (1 - np.exp(-kv*t))
    vy = vy0 * np.exp(-kv*t)
    z = z0 + ((vz0 + g/kv)/kv) * (1 - np.exp(-kv*t)) - (g*t)/kv
    vz = (vz0 + g/kv) * np.exp(-kv*t) - g/kv
    return np.array([x, y, z, vx, vy, vz])

def time_to_target(x0, vx0, target, kv):
    """
    Compute the time required for the x component to reach 'target', given:
      x(t) = x0 + (vx0/kv)*(1 - exp(-kv*t))
      
    Solving for t yields:
      t = -1/kv * ln(1 - (kv*(target - x0))/vx0)
      
    Returns None if no valid solution exists.
    """
    arg = 1 - (kv * (target - x0)) / vx0
    if arg <= 0:
        return None
    return -np.log(arg) / kv

def predict_state_at_target(initial_state, target_x, kv=0.67, g=9.81):
    """
    Given an initial 6D state [x, y, z, vx, vy, vz],
    compute the time needed for x(t) to reach target_x using the x-dynamics,
    then propagate the full state over that time using the analytical model.
    
    Returns:
      predicted_state: [x, y, z, vx, vy, vz] at the target time,
      t_target: the time required to reach target_x.
    """
    x0 = initial_state[0]
    vx0 = initial_state[3]
    t_target = time_to_target(x0, vx0, target_x, kv)
    if t_target is None:
        print("Cannot reach the target x with the given initial conditions.")
        return None, None
    predicted_state = propagate_state_analytical(initial_state, t_target, kv, g)
    return predicted_state, t_target

 
############################# Main Loop ###################################
def main():
    sock = setup_socket()
    centroid_buffer = []
    camera_buffer = []
    timestamp_buffer = []
    
    # Timer will start after the first centroid is received.
    start = None

    try:
        while True:
            result = receive_centroid(sock)
            if result:
                pi_id, cx, cy, ts_full = result

                # Print the received data immediately.
                print(f"[{ts_full}] Pi {pi_id} -> centroid_x = {cx}, centroid_y = {cy}")

                # Extract the seconds part from the timestamp (assumes format "20:19:11.365504")
                parts = ts_full.split(":")
                # Use the seconds component (with microseconds) as a float.
                ts_val = float(parts[2])
                centroid_buffer.append([cx, cy])
                camera_buffer.append(pi_id)
                timestamp_buffer.append(ts_val)
                
                # Start the timer on receiving the first measurement.
                if start is None:
                    start = time.perf_counter()
            
            # If the timer has started, check if we've collected data for at least time (s).
            if start is not None and time.perf_counter() - start > 0.200:
                break
            
            time.sleep(0.001)  # small sleep to reduce CPU usage
    
    except KeyboardInterrupt:
        print("Main loop terminated.")
    finally:
        sock.close()
    
    if len(timestamp_buffer) == 0:
        print("No centroids received.")
        return
    
    # Process the buffered data with the UKF.
    # Initialize the previous time to the first buffered time.
    prev_time = timestamp_buffer[0]
    for i in range(len(timestamp_buffer)):
        current_time = timestamp_buffer[i]
        dt = current_time - prev_time if i > 0 else 0.0
        prev_time = current_time
        
        # Perform UKF prediction.
        ukf.predict(dt)
        ukf.P = (ukf.P + ukf.P.T) / 2 + np.eye(dim_x) * 1e-6
        
        # Use the appropriate measurement function based on the camera id.
        meas = np.array(centroid_buffer[i])
        if camera_buffer[i] == 2:
            ukf.update(meas, R=R_meas, hx=hx_floor)
        elif camera_buffer[i] == 1:
            ukf.update(meas, R=R_meas, hx=hx_side)
        
        ukf.P = (ukf.P + ukf.P.T) / 2 + np.eye(dim_x) * 1e-6
    startime = time.perf_counter()    
    # After processing all buffered data, make a prediction.
    target_x = -2.10  # desired x position (in meters)
    predicted_state, t_target = predict_state_at_target(ukf.x, target_x, kv=0.67, g=9.81)
    if predicted_state is not None:
        print("Time to reach x = {:.2f} m: {:.4f} s".format(target_x, t_target))
        print("Predicted state at x = {:.2f} m:".format(target_x))
        print("x = {:.4f}, y = {:.4f}, z = {:.4f}, vx = {:.4f}, vy = {:.4f}, vz = {:.4f}".format(
            predicted_state[0]*39.37, predicted_state[1]*39.37-3.5,
            predicted_state[2]*39.37+3, predicted_state[3],
            predicted_state[4], predicted_state[5]))
    else:
        print("Prediction failed.")
    print(startime - time.perf_counter())
if __name__ == "__main__":
    main()
