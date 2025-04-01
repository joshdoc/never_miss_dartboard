# ===================================================================
# Section: Imports
# ===================================================================

import sys
import select
import serial
import time

# ===================================================================
# Section: UART Functions
# ===================================================================

def configure_port(port, baudrate=115200, timeout=1):
    try:
        # Open the port in read-write mode for bidirectional communication
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        return ser
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}")
        return None

def receive_packet(ser):
    """
    Attempts to read a 4-byte packet from the serial port.
    Returns a tuple: (reception_timestamp, centroid_x, centroid_y)
    If a full packet is not available, returns None.
    """
    data = ser.read(4)
    if len(data) == 4:
        reception_time = time.time()  # Timestamp of reception in seconds
        centroid_x = int.from_bytes(data[0:2], byteorder='little')
        centroid_y = int.from_bytes(data[2:4], byteorder='little')
        return reception_time, centroid_x, centroid_y
    return None

# ===================================================================
# Section: Main
# ===================================================================

def main():
    # Opens & Configures UART Ports
    port0 = "/dev/ttyAMA0"  # Camera 1
    port2 = "/dev/ttyAMA2"  # Camera 2
    ser0 = configure_port(port0)
    ser2 = configure_port(port2)
    
    # Error-Handling
    if not ser0 or not ser2:
        print("Failed to open one or more UART ports.")
        sys.exit(1)
    
    print("Listening for incoming UART transmissions...")
    
    # Loop to process incoming UART transmissions
    while True:
        # Monitor both serial ports
        rlist, _, _ = select.select([ser0.fileno(), ser2.fileno()], [], [])
        for fd in rlist:
            if fd == ser0.fileno():
                packet = receive_packet(ser0)
                if packet:
                    rec_time, cx, cy = packet
                    print(f"Received on {ser0.port} (Client 1): time={rec_time:.3f} s, centroid=({cx}, {cy})")
            if fd == ser2.fileno():
                packet = receive_packet(ser2)
                if packet:
                    rec_time, cx, cy = packet
                    print(f"Received on {ser2.port} (Client 2): time={rec_time:.3f} s, centroid=({cx}, {cy})")

if __name__ == "__main__":
    main()
