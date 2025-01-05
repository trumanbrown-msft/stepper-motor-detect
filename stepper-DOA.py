import usb.core
import usb.util
import time
import RPi.GPIO as GPIO
import numpy as np
from tuning import Tuning
from luma.led_matrix.device import max7219
from luma.core.interface.serial import spi, noop


# Pin configuration
DIR = 20    # Direction GPIO Pin
STEP = 21   # Step GPIO Pin
CW = 1      # Clockwise Rotation
CCW = 0     # Counterclockwise Rotation
SPR = 200   # Steps per Revolution (1.8° per step)

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)

# Setup LED matrix
serial = spi(port=0, device=0, gpio=noop())
device = max7219(serial, cascaded=4, block_orientation=-90, rotate=0, blocks_arranged_in_reverse_order=False)
print("Created LED Matrix device")

# Initialize USB device for ReSpeaker
dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)

if not dev:
    raise ValueError("ReSpeaker 4 Mic Array not found")

Mic_tuning = Tuning(dev)

# Track audio direction and point motor
try:
    current_angle = 0  # Track the motor's position
    while True:
        direction = Mic_tuning.direction
        print(f"Audio direction: {direction} degrees")

        # Calculate target angle based on direction
        target_angle = 360 - direction  # Flip direction
        step_count = int((target_angle - current_angle) / 1.8)

        # Determine rotation direction
        if step_count > 0:
            GPIO.output(DIR, CW)
        else:
            GPIO.output(DIR, CCW)

        # Rotate stepper motor (non-blocking)
        for _ in range(abs(step_count)):
            GPIO.output(STEP, GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(STEP, GPIO.LOW)
            time.sleep(0.001)

        # Update current angle
        current_angle = target_angle

        # Small delay to prevent rapid movement
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting program.")
    GPIO.cleanup()