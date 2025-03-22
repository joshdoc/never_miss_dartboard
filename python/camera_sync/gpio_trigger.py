import sys
import termios
import tty
import select
import pigpio
import time

# Hard-coded parameters
PIN1 = 17           # First GPIO pin number
PIN2 = 18           # Second GPIO pin number
PULSE_DURATION = 0.1  # Pulse duration in seconds (adjust as needed)

def get_key(timeout=0.1):
    """Non-blocking key read from stdin."""
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None

def main():
    # Initialize pigpio
    pi = pigpio.pi()
    if not pi.connected:
        sys.exit(1)
    pi.set_mode(PIN1, pigpio.OUTPUT)
    pi.set_mode(PIN2, pigpio.OUTPUT)

    # Save current terminal settings and switch to raw mode
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    print("Press SPACE to trigger the GPIO pulse. Press 'q' to quit.")

    try:
        while True:
            key = get_key()
            if key:
                if key == ' ':
                    # Trigger: set both pins high, wait, then set them low.
                    pi.write(PIN1, 1)
                    pi.write(PIN2, 1)
                    print("Triggered!")
                    time.sleep(PULSE_DURATION)
                    pi.write(PIN1, 0)
                    pi.write(PIN2, 0)
                elif key.lower() == 'q':
                    break
            time.sleep(0.01)
    finally:
        # Restore terminal settings and cleanup GPIO
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        pi.write(PIN1, 0)
        pi.write(PIN2, 0)
        pi.stop()

if __name__ == "__main__":
    main()
