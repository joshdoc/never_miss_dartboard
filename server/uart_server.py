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

def send_start_command(ser_list):
    start_cmd = "START\n".encode('ascii')
    for ser in ser_list:
        try:
            ser.write(start_cmd)
            print(f"Sent start command on {ser.port}")
        except Exception as e:
            print(f"Error sending start command on {ser.port}: {e}")

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
    
    # Wait for user input to send start command
    ports = [ser0, ser2]
    print("Press ENTER to send start capture command to cameras...")
    rlist, _, _ = select.select([sys.stdin], [], [])
    if sys.stdin in rlist:
        line = sys.stdin.readline()
        if line.strip().lower() == "start" or line.strip() == "":
            send_start_command(ports)
    
    # Loop to process incoming UART transmissions
    while True:
        rlist, _, _ = select.select([ser0.fileno(), ser2.fileno()], [], [])
        for fd in rlist:
            if fd == ser0.fileno():
                data = ser0.read(6)
                if len(data) == 6:
                    t = int.from_bytes(data[0:2], byteorder='little')
                    c1 = int.from_bytes(data[2:4], byteorder='little')
                    c2 = int.from_bytes(data[4:6], byteorder='little')
                    print(f"Received on {ser0.port} (Client 1): time={t} ms, centroid1={c1}, centroid2={c2}")
            if fd == ser2.fileno():
                data = ser2.read(6)
                if len(data) == 6:
                    t = int.from_bytes(data[0:2], byteorder='little')
                    c1 = int.from_bytes(data[2:4], byteorder='little')
                    c2 = int.from_bytes(data[4:6], byteorder='little')
                    print(f"Received on {ser2.port} (Client 2): time={t} ms, centroid1={c1}, centroid2={c2}")

if __name__ == "__main__":
    main()
