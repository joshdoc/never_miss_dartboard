import socket
import time
import select

DEST_PORT = 4600  # Must match the port used by the transmitter

def setup_socket():
    # Create and bind the UDP socket to all interfaces on the allowed port
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('', DEST_PORT))
    # Set to non-blocking mode
    s.setblocking(0)
    return s

def receive_centroids(s):
    # Use select with a very short timeout to avoid blocking
    readable, _, _ = select.select([s], [], [], 0.001)  # timeout in seconds (1 ms)
    if s in readable:
        data, addr = s.recvfrom(1024)  # buffer size is more than enough (expecting 4 bytes)
        if len(data) == 4:
            # Unpack the two uint16 values (big-endian network order)
            centroid1 = int.from_bytes(data[0:2], byteorder='big')
            centroid2 = int.from_bytes(data[2:4], byteorder='big')
            timestamp = time.time()  # record the receipt timestamp
            # Print out the centroids, timestamp, and sender IP to distinguish RPIs
            print(f"Received from {addr[0]}: centroid1 = {centroid1}, centroid2 = {centroid2} at {timestamp:.6f}")
        else:
            print(f"Received unexpected data length: {len(data)} bytes")

def main():
    s = setup_socket()
    print(f"Listening for UDP packets on port {DEST_PORT}...")
    try:
        while True:
            receive_centroids(s)
            # Minimal sleep to yield CPU; adjust as needed to reduce impact on latency
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("Exiting receiver.")
    finally:
        s.close()

if __name__ == "__main__":
    main()
