import time
import click
from gpiozero import OutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory
import logging

# Configuration
TRIGGER_PINS = [17, 18]
PULSE_DURATION = 0.1  # 100ms pulse

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler()]
)

class StereoTriggerServer:
    def __init__(self):
        self.logger = logging.getLogger("Server")
        self._setup_gpio()

    def _setup_gpio(self):
        factory = PiGPIOFactory()
        self.triggers = [
            OutputDevice(pin, active_high=True, initial_value=False, pin_factory=factory)
            for pin in TRIGGER_PINS
        ]
        self.logger.info(f"Trigger pins initialized: {TRIGGER_PINS}")

    def send_pulse(self):
        """Sends a synchronized trigger pulse to both GPIO pins."""
        try:
            self.logger.info("Sending synchronized trigger pulse")
            for trigger in self.triggers:
                trigger.on()
            time.sleep(PULSE_DURATION)
            for trigger in self.triggers:
                trigger.off()
            self.logger.info("Trigger pulse complete")
        except Exception as e:
            self.logger.error(f"Trigger error: {str(e)}")

@click.command()
@click.option("--test", is_flag=True, help="Test mode (auto-trigger every 2s)")
def main(test):
    server = StereoTriggerServer()
    logging.info("Stereo Trigger Server Ready")
    
    try:
        if test:
            logging.info("Entering test mode...")
            while True:
                server.send_pulse()
                time.sleep(2)
        else:
            click.echo("Press ENTER to trigger cameras (CTRL+C to exit)")
            while True:
                input()
                server.send_pulse()
    except KeyboardInterrupt:
        logging.info("Server shutdown complete")

if __name__ == "__main__":
    main()
