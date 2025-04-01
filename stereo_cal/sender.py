#!/usr/bin/env python3
import time
import serial
from picamera import PiCamera

COUNT_FILE = "capture_count.txt"

def configure_uart(port="/dev/ttyAMA0", baudrate=115200, timeout=1):
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        return ser
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}")
        return None

def get_next_capture_index():
    """Reads the last capture index from file and increments it."""
    try:
        with open(COUNT_FILE, "r") as f:
            index = int(f.read().strip())
    except (FileNotFoundError, ValueError):
        index = 0  # Start from 0 if no previous count exists
    
    index += 1  # Increment for next capture
    
    with open(COUNT_FILE, "w") as f:
        f.write(str(index))  # Save updated index
    
    return index

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

    print("Press ENTER to capture an image on both Pis...")
    while True:
        input("Press ENTER when ready: ")

        capture_index = get_next_capture_index()
        
        # Send capture index over UART
        trigger_command = f"CAPTURE,{capture_index}\n".encode('ascii')
        ser.write(trigger_command)
        
        # Capture image on master Pi
        filename = capture_image(camera, capture_index)
        print(f"[Master] Image captured and saved as '{filename}'.")
        print("Capture complete. Ready for next input.\n")

if __name__ == "__main__":
    main()
