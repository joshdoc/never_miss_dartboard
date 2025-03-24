import serial
import time

# Configure UART
SERIAL_PORT = "/dev/serial0"  # Change based on setup
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Allow serial to initialize

while True:
    timestamp = time.time()  # Get current time
    message = f"{timestamp}\n"
    
    ser.write(message.encode())  # Send timestamp
    print(f"Sent: {message.strip()}")

    time.sleep(1)  # Adjust as needed
