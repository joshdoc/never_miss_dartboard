import serial
import time

# Open serial port for writing (AMA0)
ser = serial.Serial("/dev/ttyAMA0", baudrate=115200, timeout=1)

try:
    while True:
        sensor_data = 1234  # Example number to send
        data_bytes = sensor_data.to_bytes(2, byteorder='little')  # Convert to 2-byte format
        ser.write(data_bytes)  # Send data
        print(f"Sent: {sensor_data}")
        time.sleep(1)  # Wait 1 second before sending again
except KeyboardInterrupt:
    print("\nClient stopped.")
finally:
    ser.close()
