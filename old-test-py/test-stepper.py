import RPi.GPIO as GPIO
from time import sleep
import random

# Pin configuration
DIR = 20    # Direction GPIO Pin
STEP = 21   # Step GPIO Pin
CW = 1      # Clockwise Rotation
CCW = 0     # Counterclockwise Rotation
SPR = 200   # Steps per Revolution (1.8° per step for finer control)

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)

# Start settings
step_count = SPR
delay = 0.001  # Start with a very fast delay

direction = CW  # Start rotating clockwise

try:
    while True:
        for x in range(step_count):
            GPIO.output(DIR, direction)
            GPIO.output(STEP, GPIO.HIGH)
            sleep(delay)  # Adjusted to delay for high speed
            GPIO.output(STEP, GPIO.LOW)
            sleep(delay)

        # Randomly switch direction with lower probability (e.g., 1 in 10 chances)
        if random.random() < 0.1:  # 10% chance to reverse direction
            direction = CW if direction == CCW else CCW

        # Change speed drastically in larger intervals, but not too frequently
        delay_factor = random.uniform(0.5, 1.5)  # Randomly choose factor between 0.5 and 1.5
        delay *= delay_factor

        # Limit the delay to a reasonable range, allowing maximum speed
        if delay < 0.0001:
            delay = 0.0001  # Minimum delay for max speed
        elif delay > 0.02:
            delay = 0.02  # Upper bound to avoid running too slow

        # Print delay for debugging
        print(f"Current delay: {delay:.6f} seconds")

except KeyboardInterrupt:
    GPIO.cleanup()
