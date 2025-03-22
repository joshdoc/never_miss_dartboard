import pigpio
import time

# Hard-coded parameters
PIN1 = 17           # First GPIO pin number
PIN2 = 18           # Second GPIO pin number
FREQUENCY = 70.0    # Frequency in Hz

# Calculate half period in seconds
half_period = (1.0 / FREQUENCY) / 2

pi = pigpio.pi()    # Connect to the local pigpio daemon
if not pi.connected:
    exit(1)

pi.set_mode(PIN1, pigpio.OUTPUT)
pi.set_mode(PIN2, pigpio.OUTPUT)

while True:
    # Set both pins high
    pi.write(PIN1, 1)
    pi.write(PIN2, 1)
    time.sleep(half_period)

    # Set both pins low
    pi.write(PIN1, 0)
    pi.write(PIN2, 0)
    time.sleep(half_period)
