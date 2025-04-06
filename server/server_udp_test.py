import socket
import struct
import time
from datetime import datetime

DEST_PORT = 4600  # Port must match sender

def setup_socket(port=DEST_PORT):
    """Set up and return a blocking UDP socket bound to the given port."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', port))
    sock.setblocking(False)  # Non-blocking for integration into main loop
    return sock

def receive_centroid(sock):
    """
    Attempts to receive a 4-byte UDP packet.
    Returns:
        (cx, cy, timestamp) if a valid packet is received
        None if no packet is available
    """
    try:
        data, addr = sock.recvfrom(1024)
        if len(data) == 4:
            cx, cy = struct.unpack('!HH', data)
            timestamp = datetime.now().isoformat(timespec='microseconds')
            return cx, cy, timestamp
        else:
            print(f"Received invalid packet length: {len(data)} from {addr}")
            return None
    except BlockingIOError:
        return None  # No data available this cycle

def main():
    sock = setup_socket()

    try:
        while True:
            result = receive_centroid(sock)
            if result:
                cx, cy, ts = result
                print(f"[{ts}] Received: centroid_x = {cx}, centroid_y = {cy}")
            
            # You can integrate other tasks here:
            # update_display()
            # handle_user_input()
            # run_ai_logic()
            
            time.sleep(0.005)  # small sleep to reduce CPU usage
    except KeyboardInterrupt:
        print("Main loop terminated.")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
