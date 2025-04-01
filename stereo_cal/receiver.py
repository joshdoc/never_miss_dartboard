#!/usr/bin/env python3
import time
import serial
from picamera import PiCamera

def configure_uart(port="/dev/ttyAMA0", baudrate=115200, timeout=1):
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        return ser
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}")
        return None

def capture_image(camera, index):
    filename = f"capture_{index}.jpg"
    camera.capture(filename)
    return filename

def main():
    ser = configure_uart()
    if not ser:
        return

    camera = PiCamera()
    camera.resolution = (1280, 720)
    time.sleep(2)  # Camera warm-up time

    print("Waiting for capture command via UART on /dev/ttyAMA0...")

    while True:
        try:
            line = ser.readline().decode('ascii').strip()
        except Exception as e:
            print(f"Error reading UART: {e}")
            continue

        if line.startswith("CAPTURE,"):
            try:
                capture_index = int(line.split(",")[1])
            except ValueError:
                print("Invalid capture command received.")
                continue

            filename = capture_image(camera, capture_index)
            print(f"[Slave] Image captured and saved as '{filename}'.")
            print("Capture complete. Waiting for next command...\n")

if __name__ == "__main__":
    main()
