import time
from datetime import datetime
from pathlib import Path
from gpiozero import Button
from gpiozero.pins.pigpio import PiGPIOFactory
import subprocess
import logging

# Configuration
TRIGGER_PIN = 17
IMAGE_DIR = Path.home() / "Desktop" / "StereoCaptures"
RESOLUTION = (1280, 720)
DEBOUNCE_SEC = 0.1  # 100ms debounce time

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler()]
)

class StereoCameraClient:
    def __init__(self):
        self.logger = logging.getLogger('Client')
        self._setup_gpio()
        self._setup_directory()
        self.capture_lock = False

    def _setup_directory(self):
        IMAGE_DIR.mkdir(parents=True, exist_ok=True)
        self.logger.info(f"Image directory: {IMAGE_DIR}")

    def _setup_gpio(self):
        factory = PiGPIOFactory()  # Uses pigpio daemon but with GPIO Zero API
        self.trigger = Button(
            TRIGGER_PIN,
            pull_up=False,
            bounce_time=DEBOUNCE_SEC,
            pin_factory=factory
        )
        self.trigger.when_pressed = self._capture_handler

    def _capture_image(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = IMAGE_DIR / f"stereo_{timestamp}.jpg"
        
        cmd = [
            "libcamera-jpeg",
            "-o", str(filename),
            "--width", str(RESOLUTION[0]),
            "--height", str(RESOLUTION[1]),
            "--nopreview",
            "--timeout", "1"
        ]
        
        try:
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            self.logger.info(f"Captured {filename.name}")
            self.logger.debug(f"Camera output: {result.stdout}")
        except subprocess.CalledProcessError as e:
            self.logger.error(f"Capture failed: {e.stderr}")

    def _capture_handler(self):
        if not self.capture_lock:
            self.capture_lock = True
            try:
                self.logger.info("Trigger received - Starting capture")
                self._capture_image()
            finally:
                self.capture_lock = False
        else:
            self.logger.warning("Capture already in progress - Ignoring trigger")

if __name__ == "__main__":
    client = StereoCameraClient()
    logging.info("Stereo Camera Client Ready")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("Client shutdown complete")
