import serial

# Open serial port for reading (AMA0)
ser = serial.Serial("/dev/ttyAMA0", baudrate=115200, timeout=1)

try:
    print("Waiting for data...")

    while True:
        data = ser.read(2)  # Read 2 bytes
        if len(data) == 2:  # Ensure full data is received
            sensor_data = int.from_bytes(data, byteorder='little')  # Convert bytes to int
            print(f"Received: {sensor_data}")
except KeyboardInterrupt:
    print("\nServer stopped.")
finally:
    ser.close()
