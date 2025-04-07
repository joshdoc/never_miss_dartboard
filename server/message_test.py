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


############################# Main Loop ###################################
def main():
    sock = setup_socket()
    first_meas = True
    try:
        while True:
            result = receive_centroid(sock)
            
            if result:
                pi_id, cx, cy, ts = result

              

                print(f"[{ts}] Pi {pi_id} -> centroid_x = {cx}, centroid_y = {cy}")

               
            time.sleep(0.003)  # small sleep to reduce CPU usage
    
        
            
    except KeyboardInterrupt:
        print("Main loop terminated.")
    finally:
        sock.close()
    
    # propagate state forward 
    
if __name__ == "__main__":
    main()