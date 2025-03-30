# ===================================================================
# Section: Configuration Options
# ===================================================================
DEBUG = True  # Set to True to enable debug console output

# ===================================================================
# Section: Imports
# ===================================================================
import sys
import select
import serial
import time
import numpy as np
import cv2

from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

# ===================================================================
# Section: UART Functions
# ===================================================================
def configure_port(port, baudrate=115200, timeout=1):
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        if DEBUG:
            print(f"UART port {port} configured successfully.")
        return ser
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}")
        return None

def send_start_command(ser):
    start_cmd = "START\n".encode('ascii')
    try:
        ser.write(start_cmd)
        if DEBUG:
            print(f"Sent start command on {ser.port}")
    except Exception as e:
        print(f"Error sending start command on {ser.port}: {e}")

# ===================================================================
# Section: Filtering Functions
# ===================================================================
def fx(state, dt):
    """
    State transition function.
    State: [x, z, vx, vz, Y]
    """
    x, z, vx, vz, Y = state
    kv = 0.67  # drag coefficient
    g = 9.81   # gravity

    # Compute accelerations
    ax = -kv * vx
    az = -kv * vz - g

    # Euler integration for velocity
    vx_new = vx + ax * dt
    vz_new = vz + az * dt

    # Euler integration for position
    x_new = x + vx * dt
    z_new = z + vz * dt

    return np.array([x_new, z_new, vx_new, vz_new, Y])

# Measurement function: maps (x, z) to pixel coordinates (u, v)
cx, cy = 320, 240   # principal point (pixels)
f = 800             # focal length (pixels)
def hx(state):
    x, z, vx, vz, Y = state
    u = cx + (f * x) / Y
    v = cy + (f * z) / Y
    return np.array([u, v])

def propagate_state_analytical(state, t, kv=0.67, g=9.81):
    """
    Analytically propagate state over time t.
    State: [x, z, vx, vz, Y]. Y remains constant.
    """
    x0, z0, vx0, vz0, Y0 = state
    x = x0 + (vx0 / kv) * (1 - np.exp(-kv * t))
    z = z0 + ((vz0 + g / kv) / kv) * (1 - np.exp(-kv * t)) - (g * t) / kv
    vx = vx0 * np.exp(-kv * t)
    vz = (vz0 + g / kv) * np.exp(-kv * t) - g / kv
    return np.array([x, z, vx, vz, Y0])

# ===================================================================
# Section: UKF Initialization
# ===================================================================
points = MerweScaledSigmaPoints(n=5, alpha=0.01, beta=2, kappa=0)
ukf = UKF(dim_x=5, dim_z=2, fx=fx, hx=hx, dt=0.01, points=points)
# Initial state: [x, z, vx, vz, Y]
ukf.x = np.array([0, 0, 8 * np.cos(np.radians(5)), 8 * np.sin(np.radians(5)), 1.83])
ukf.P *= 0.3
ukf.R *= 6
ukf.Q *= 0.2
ukf.Q[4, 4] = 0.01

x_final = 2.5  # The x position where we predict z
latest_dt = 0.01

# ===================================================================
# Section: UART Initialization
# ===================================================================
port0 = "/dev/ttyAMA0"  # Camera 1 (only this channel is used)
ser0 = configure_port(port0)

if not ser0:
    print("Failed to open UART port.")
    sys.exit(1)

print("Press ENTER to send start capture command to camera...")
rlist, _, _ = select.select([sys.stdin], [], [])
if sys.stdin in rlist:
    line = sys.stdin.readline()
    if line.strip().lower() == "start" or line.strip() == "":
        send_start_command(ser0)

# ===================================================================
# Section: Main Loop
# ===================================================================
def main():
    msg_cnt = 0
    start_time = None

    running = True
    while running:
        # Wait briefly for UART data from ser0
        rlist, _, _ = select.select([ser0.fileno()], [], [], 0.01)
        for fd in rlist:
            if fd == ser0.fileno():
                data = ser0.read(6)
                if len(data) == 6:
                    # Parse the 6-byte message from the UART
                    t = int.from_bytes(data[0:2], byteorder='little')
                    c1 = int.from_bytes(data[2:4], byteorder='little')
                    c2 = int.from_bytes(data[4:6], byteorder='little')
                    print(f"Received on {ser0.port}: time={t} ms, centroid1={c1}, centroid2={c2}")
                    
                    if msg_cnt == 0:
                        start_time = time.time()
                    
                    latest_dt = t / 1000.0
                    pixel_measurement = (c1, c2)
                    ukf.predict(dt=latest_dt)
                    ukf.update(pixel_measurement)
                    
                    elapsed_time = (time.time() - start_time) * 1000 if start_time else 0
                    print(f"Elapsed: {elapsed_time:.2f} ms, Estimated State: {ukf.x}")
                    
                    msg_cnt += 1

        # Exit loop after a specified elapsed time (e.g., 175 ms)
        if start_time and (time.time() - start_time) * 1000 >= 175:
            running = False

    # After processing, use analytical propagation to predict state at x_final.
    propagated_state = propagate_state_analytical(ukf.x, x_final)
    z_final = propagated_state[1]
    print(f"Predicted Height at x={x_final}m: {z_final:.3f}m")

if __name__ == "__main__":
    main()

sys.exit()
