import serial
import time

# Configure UART
SERIAL_PORT = "/dev/serial0"  # Change based on setup
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Allow serial to initialize

while True:
    data = ser.readline().decode().strip()
    
    if data:
        sent_time = float(data)  # Convert back to float
        received_time = time.time()
        latency = received_time - sent_time  # Calculate transmission delay

        print(f"Received: {sent_time}, Latency: {latency:.6f} sec")
