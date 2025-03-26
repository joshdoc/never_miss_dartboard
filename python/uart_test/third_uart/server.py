import serial
import select

def configure_port(port, baudrate=115200, timeout=1):
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        return ser
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}")
        return None

def main():
    # Open two UART ports: one for each client
    port0 = "/dev/ttyAMA0"  # Connected to Client 1
    port2 = "/dev/ttyAMA2"  # Connected to Client 2

    ser0 = configure_port(port0)
    ser2 = configure_port(port2)

    if not ser0 or not ser2:
        print("Failed to open one or more UART ports.")
        return

    print(f"Server listening on {port0} (Client 1) and {port2} (Client 2)...")
    
    # Use file descriptors from the serial objects for select
    while True:
        # Wait for data on either port
        rlist, _, _ = select.select([ser0.fileno(), ser2.fileno()], [], [])
        for fd in rlist:
            if fd == ser0.fileno():
                data = ser0.read(2)  # Read 2 bytes
                if len(data) == 2:
                    value = int.from_bytes(data, byteorder='little')
                    print(f"Received on {port0} (Client 1): {value}")
            if fd == ser2.fileno():
                data = ser2.read(2)  # Read 2 bytes
                if len(data) == 2:
                    value = int.from_bytes(data, byteorder='little')
                    print(f"Received on {port2} (Client 2): {value}")

if __name__ == "__main__":
    main()
