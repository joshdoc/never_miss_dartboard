import socket

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('', 4500))
print("Listening on port 4500...")

while True:
    data, addr = s.recvfrom(1024)
    print(f"Received {len(data)} bytes from {addr}: {data}")
