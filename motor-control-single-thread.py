import usb.core
import usb.util
import time
import RPi.GPIO as GPIO
from tuning import Tuning
import threading

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

# Initialize USB device for ReSpeaker
dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)

if not dev:
    raise ValueError("ReSpeaker 4 Mic Array not found")

Mic_tuning = Tuning(dev)

# Motor control function
def motor_control():
    current_angle = 0  # Track the motor's position
    while True:
        direction = Mic_tuning.direction
        print(f"Audio direction: {direction} degrees")

        # Calculate target angle based on direction
        target_angle = 360 - direction  # Flip direction
        target_angle = target_angle % 360  # Normalize the target angle

        # Calculate the difference in angles
        angle_diff = target_angle - current_angle

        # Ensure angle_diff is between -180 and 180 (shortest path)
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360

        # Determine rotation direction (shortest path)
        if angle_diff > 0:
            GPIO.output(DIR, CW)  # Clockwise
        else:
            GPIO.output(DIR, CCW)  # Counterclockwise

        # Rotate stepper motor
        step_count = int(abs(angle_diff) / 1.8)  # Steps per revolution (1.8° per step)
        for _ in range(step_count):
            GPIO.output(STEP, GPIO.HIGH)
            time.sleep(0.001)  # Short delay for step
            GPIO.output(STEP, GPIO.LOW)
            time.sleep(0.001)  # Short delay for step

        # Update current angle
        current_angle = target_angle

        # Small delay to prevent rapid movement
        time.sleep(0.04)

# Start the motor control in a separate thread
motor_thread = threading.Thread(target=motor_control)
motor_thread.daemon = True  # Daemonize the thread so it exits when the main program exits
motor_thread.start()

try:
    while True:
        time.sleep(1)  # Main thread does nothing but keeps the program running
except KeyboardInterrupt:
    print("Exiting program.")
    GPIO.cleanup()
