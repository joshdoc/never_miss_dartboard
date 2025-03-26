import serial
import time

def main():
    port = "/dev/ttyAMA0"  # Client 2 uses AMA0 as well (connected to server's AMA2)
    try:
        ser = serial.Serial(port, baudrate=115200, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}")
        return

    print(f"Client 2 transmitting on {port}...")
    try:
        while True:
            sensor_data = 5678  # Example value for Client 2
            data_bytes = sensor_data.to_bytes(2, byteorder='little')
            ser.write(data_bytes)
            print(f"Client 2 Sent: {sensor_data}")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nClient 2 stopped.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
