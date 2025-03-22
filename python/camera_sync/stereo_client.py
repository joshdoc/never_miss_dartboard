import pigpio
import os
import time
from datetime import datetime

# Configuration
TRIGGER_PIN = 17         # GPIO pin to monitor
IMAGE_DIR = "/home/pi/stereo_images"
RESOLUTION = (1280, 720)  # 720p mode
CAPTURE_CMD = (
    f"libcamera-jpeg -o {{}} "
    f"--width {RESOLUTION[0]} --height {RESOLUTION[1]} "
    "--nopreview --timeout 1"
)

pi = pigpio.pi()
pi.set_mode(TRIGGER_PIN, pigpio.INPUT)
pi.set_pull_up_down(TRIGGER_PIN, pigpio.PUD_DOWN)

def capture_image():
    """Capture 720p image with timestamp"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = os.path.join(IMAGE_DIR, f"stereo_{timestamp}.jpg")
    os.system(CAPTURE_CMD.format(filename))
    print(f"Captured 720p image: {filename}")

def trigger_callback(gpio, level, tick):
    """Hardware-timed trigger response"""
    if level == 1:
        # Add small delay to ensure camera readiness
        time.sleep(0.02)  
        capture_image()

# Setup environment
os.makedirs(IMAGE_DIR, exist_ok=True)
pi.callback(TRIGGER_PIN, pigpio.RISING_EDGE, trigger_callback)

try:
    print(f"Ready for 720p captures on GPIO {TRIGGER_PIN}")
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pi.stop()
