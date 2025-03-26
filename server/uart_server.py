import serial
import select

def configure_port(port, baudrate=115200):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        ser.flush()
        return ser
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}")
        return None

def main():
    port0 = "/dev/ttyAMA0"
    port2 = "/dev/ttyAMA2"
    
    ser0 = configure_port(port0)
    ser2 = configure_port(port2)
    
    if not ser0 or not ser2:
        print("Failed to open one or more UART ports.")
        return
    
    print(f"Listening on {port0} and {port2}...")
    
    while True:
        readable, _, _ = select.select([ser0, ser2], [], [])
        
        for ser in readable:
            data = ser.read(2)  # Reading 2 bytes (uint16_t)
            if len(data) == 2:
                sensor_data = int.from_bytes(data, byteorder='little')
                print(f"Received on {ser.port}: {sensor_data}")

if __name__ == "__main__":
    main()
