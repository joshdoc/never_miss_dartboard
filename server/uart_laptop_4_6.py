# server_uart.py
import serial
import time

def open_serial(port, baudrate=115200):
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1)
        print(f"Opened {port} successfully.")
        return ser
    except serial.SerialException as e:
        print(f"Failed to open {port}: {e}")
        return None

def main():
    port = "COM3"  # Replace with the COM port of your adapter
    ser = open_serial(port)

    if not ser:
        return

    print("Listening for incoming data...")

    while True:
        data = ser.read(4)
        if len(data) == 4:
            x = int.from_bytes(data[0:2], byteorder='little')
            y = int.from_bytes(data[2:4], byteorder='little')
            print(f"Received: x={x}, y={y}")
        else:
            print("Waiting for data...")
        time.sleep(0.1)

if __name__ == "__main__":
    main()
