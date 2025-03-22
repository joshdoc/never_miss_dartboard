import os
import pigpio
import time
from datetime import datetime

# Configuration
TRIGGER_PIN = 17  # GPIO pin to monitor
DESKTOP_DIR = os.path.join(os.path.expanduser("~"), "Desktop")
IMAGE_DIR = DESKTOP_DIR  # Save images to the Desktop
RESOLUTION = (1280, 720)      # Desired resolution (width, height)

# Ensure the image directory exists (should always exist for Desktop)
os.makedirs(IMAGE_DIR, exist_ok=True)

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpio daemon")
    exit(1)

# Setup GPIO pin
pi.set_mode(TRIGGER_PIN, pigpio.INPUT)
pi.set_pull_up_down(TRIGGER_PIN, pigpio.PUD_DOWN)  # Hardware stabilization
pi.set_glitch_filter(TRIGGER_PIN, 100000)  # 100ms glitch filter

def capture_image(timestamp):
    """Atomic image capture function using libcamera-jpeg"""
    filename = os.path.join(
        IMAGE_DIR,
        f"capture_{timestamp:.6f}.jpg"
    )
    cmd = (
        f"libcamera-jpeg -o {filename} "
        f"--width {RESOLUTION[0]} --height {RESOLUTION[1]} "
        "--nopreview --timeout 1"
    )
    os.system(cmd)
    print(f"Captured: {filename}")

def trigger_callback(gpio, level, tick):
    if level == 1:
        # Use the current time (as float seconds) as a timestamp
        ts = time.time()
        formatted_time = datetime.fromtimestamp(ts).strftime("%H:%M:%S.%f")[:-3]
        print(f"[{formatted_time}] HIGH pulse detected on GPIO{gpio}")
        capture_image(ts)

# Register callback for rising edge detection
pi.callback(TRIGGER_PIN, pigpio.RISING_EDGE, trigger_callback)

try:
    print("Pulse detector running...")
    print(f"Monitoring GPIO{TRIGGER_PIN}")
    print("Press CTRL+C to exit")
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStopping detector...")
finally:
    pi.stop()
