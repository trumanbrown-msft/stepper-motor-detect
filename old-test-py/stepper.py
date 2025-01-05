import RPi.GPIO as GPIO
from time import sleep

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
delay = 0.002  # Start with a reasonable delay for faster rotation

# Define movement sequence (duration in seconds) and directions
phases = [
    {"duration": 5, "direction": CW, "speed_factor": 0.85},  # 5 seconds clockwise, speeding up
    {"duration": 5, "direction": CCW, "speed_factor": 1.15},  # 5 seconds counterclockwise, speeding up
    {"duration": 5, "direction": CW, "speed_factor": 0.9},   # 5 seconds clockwise, slowing down
    {"duration": 5, "direction": CCW, "speed_factor": 1.0},   # 5 seconds counterclockwise at stable speed
]

# Start sequence of movements
try:
    start_time = 0
    for phase in phases:
        duration = phase["duration"]
        direction = phase["direction"]
        speed_factor = phase["speed_factor"]
        
        end_time = start_time + duration
        delay = delay * speed_factor  # Adjust delay based on speed factor

        # Ensure that delay does not go too fast (max speed)
        if delay < 0.0005:
            delay = 0.0005  # Minimum delay for max speed
        elif delay > 0.005:
            delay = 0.005  # Upper bound to avoid running too slow

        # Rotate motor for the phase duration
        GPIO.output(DIR, direction)
        while start_time < end_time:
            GPIO.output(STEP, GPIO.HIGH)
            sleep(delay)  # Adjusted delay for speed control
            GPIO.output(STEP, GPIO.LOW)
            sleep(delay)

            start_time += delay * 2  # Increment time based on delay

        # After phase, switch to next phase
        start_time = end_time  # Move to next phase

except KeyboardInterrupt:
    GPIO.cleanup()
