import pigpio
import time
import sys

# Configuration
TRIGGER_PINS = [17, 18]  # GPIO17 and GPIO18
PULSE_DURATION = 0.1      # 100ms pulse width

pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpio daemon")
    sys.exit(1)

# Setup GPIO outputs
for pin in TRIGGER_PINS:
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.write(pin, 0)

def send_trigger():
    """Send synchronized trigger pulse to both cameras"""
    try:
        # Set both pins high
        pi.write(TRIGGER_PINS[0], 1)
        pi.write(TRIGGER_PINS[1], 1)
        time.sleep(PULSE_DURATION)
        
        # Set both pins low
        pi.write(TRIGGER_PINS[0], 0)
        pi.write(TRIGGER_PINS[1], 0)
        print(f"Trigger sent at {time.time():.6f}")
    except Exception as e:
        print(f"Error sending trigger: {str(e)}")

try:
    print("Stereo Capture Controller")
    print("Press ENTER to capture a stereo pair")
    print("Press CTRL+C to exit\n")
    
    while True:
        input()  # Wait for Enter key press
        send_trigger()
        
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    for pin in TRIGGER_PINS:
        pi.write(pin, 0)
    pi.stop()
