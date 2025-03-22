import pigpio
import os
import time
from datetime import datetime

# Configuration
TRIGGER_PIN = 17
IMAGE_DIR = "/home/pi/stereo_images"
RESOLUTION = (1280, 720)
DEBOUNCE_US = 500000  # 500ms cooldown (0.5 seconds)
LAST_TRIGGER = 0

pi = pigpio.pi()
if not pi.connected:
    exit("Failed to connect to pigpio daemon")

# Hardware stabilization
pi.set_mode(TRIGGER_PIN, pigpio.INPUT)
pi.set_pull_up_down(TRIGGER_PIN, pigpio.PUD_DOWN)
pi.set_glitch_filter(TRIGGER_PIN, 10000)  # 10ms glitch filter
pi.set_noise_filter(TRIGGER_PIN, 5000, 20000)  # Stable for 5ms, max wait 20ms

def capture_image(timestamp):
    """Atomic image capture function"""
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
    """Hardware-validated trigger handler"""
    global LAST_TRIGGER
    
    # Validate trigger conditions
    current_time = pi.get_current_tick()
    if all([
        level == 1,  # Rising edge
        (current_time - LAST_TRIGGER) > DEBOUNCE_US,  # Debounce check
        pi.read(TRIGGER_PIN) == 1  # Current pin state verification
    ]):
        LAST_TRIGGER = current_time
        capture_time = current_time / 1e6  # Convert μs to seconds
        capture_image(capture_time)

# Setup environment
os.makedirs(IMAGE_DIR, exist_ok=True)
cb = pi.callback(TRIGGER_PIN, pigpio.RISING_EDGE, trigger_callback)

try:
    print("Client ready - Strict trigger validation enabled")
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    cb.cancel()
    pi.stop()
