import socket
import sys

UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 4500

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP messages on port {UDP_PORT} (Press Ctrl+C to exit)...")
try:
    while True:
        data, addr = sock.recvfrom(1024)
        print(f"Received: {data.decode()} from {addr}")
except KeyboardInterrupt:
    print("\nExiting...")
    sock.close()
    sys.exit(0)