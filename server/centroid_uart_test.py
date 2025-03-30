# ===================================================================
# Section: Configuration Options
# ===================================================================
DEBUG = True  # Set to True to enable debug console output

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
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        if DEBUG:
            print(f"UART port {port} configured successfully.")
        return ser
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}")
        return None

def send_start_command(ser):
    start_cmd = "START\n".encode('ascii')
    try:
        ser.write(start_cmd)
        if DEBUG:
            print(f"Sent start command on {ser.port}")
    except Exception as e:
        print(f"Error sending start command on {ser.port}: {e}")

# ===================================================================
# Section: UART Initialization
# ===================================================================
port0 = "/dev/ttyAMA2"  # Using only this channel for initial testing
ser0 = configure_port(port0)

if not ser0:
    print("Failed to open UART port.")
    sys.exit(1)

print("Press ENTER to send start capture command to camera...")
rlist, _, _ = select.select([sys.stdin], [], [])
if sys.stdin in rlist:
    line = sys.stdin.readline()
    if line.strip().lower() == "start" or line.strip() == "":
        send_start_command(ser0)

# ===================================================================
# Section: Main Loop (Continuous Centroid Detection Debug)
# ===================================================================
def main():
    msg_cnt = 0
    start_time = None

    while True:  # Continuous loop, no exit condition
        # Wait briefly for UART data from ser0
        rlist, _, _ = select.select([ser0.fileno()], [], [], 0.01)
        for fd in rlist:
            if fd == ser0.fileno():
                data = ser0.read(6)
                if len(data) == 6:
                    # Parse the 6-byte message from the UART
                    t = int.from_bytes(data[0:2], byteorder='little')
                    c1 = int.from_bytes(data[2:4], byteorder='little')
                    c2 = int.from_bytes(data[4:6], byteorder='little')

                    if msg_cnt == 0:
                        start_time = time.time()

                    elapsed_time = (time.time() - start_time) * 1000 if start_time else 0

                    # Print the received timestamp and centroids for debugging
                    print(f"Message {msg_cnt+1}: Time={t} ms, Elapsed={elapsed_time:.2f} ms, Centroid1={c1}, Centroid2={c2}")

                    msg_cnt += 1

if __name__ == "__main__":
    main()
