import pigpio
import time

# Hard-coded parameters
PIN = 17             # GPIO pin number
FREQUENCY = 70.0     # Frequency in Hz

# Calculate half period in seconds
half_period = (1.0 / FREQUENCY) / 2

pi = pigpio.pi()     # Connect to the local pigpio daemon
if not pi.connected:
    exit(1)

pi.set_mode(PIN, pigpio.OUTPUT)

while True:
    pi.write(PIN, 1)   # Set pin high
    time.sleep(half_period)
    pi.write(PIN, 0)   # Set pin low
    time.sleep(half_period)
