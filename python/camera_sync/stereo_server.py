import pigpio
import keyboard  # Requires root/sudo
import time

# Configuration
TRIGGER_PINS = [17, 18]  # GPIO pins for both cameras
PULSE_DURATION = 0.1      # 100ms pulse width

pi = pigpio.pi()
if not pi.connected:
    exit(1)

# Setup GPIO
for pin in TRIGGER_PINS:
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.write(pin, 0)  # Initialize low

def send_trigger():
    """Send synchronized trigger pulse to both cameras"""
    try:
        # Rising edge
        pi.write(TRIGGER_PINS[0], 1)
        pi.write(TRIGGER_PINS[1], 1)
        time.sleep(PULSE_DURATION)
        
        # Falling edge
        pi.write(TRIGGER_PINS[0], 0)
        pi.write(TRIGGER_PINS[1], 0)
        
        print(f"Trigger sent at {time.time():.6f}")
    except Exception as e:
        print(f"Trigger error: {str(e)}")

try:
    print("Press SPACE to capture stereo pair. ESC to exit.")
    keyboard.on_press_key('space', lambda _: send_trigger())
    keyboard.wait('esc')
    
except KeyboardInterrupt:
    pass
finally:
    for pin in TRIGGER_PINS:
        pi.write(pin, 0)
    pi.stop()
