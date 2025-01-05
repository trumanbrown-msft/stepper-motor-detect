import usb.core
import usb.util
import time
import RPi.GPIO as GPIO
import pyaudio
import numpy as np
from tuning import Tuning
from time import sleep
import threading
from luma.led_matrix.device import max7219
from luma.core.interface.serial import spi, noop
from luma.core.legacy import show_message
from luma.core.legacy.font import proportional, CP437_FONT, LCD_FONT


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

# Set up PyAudio for real-time audio capture
p = pyaudio.PyAudio()

# Parameters for audio capture
rate = 16000  # Sample rate for ReSpeaker 4-Mic Array
chunk = 24000  # Chunk size (1.5 seconds)
overlap = 18000  # 75% overlap (1.125 seconds overlap)
channels = 1  # Mono audio
format = pyaudio.paInt16
THRESHOLD = 0.1  # Threshold for noise detection

# Open the audio stream
stream = p.open(format=format,
                channels=channels,
                rate=rate,
                input=True,
                frames_per_buffer=chunk)

# Function to calculate volume from raw audio data
def calculate_volume(data):
    audio_data = np.frombuffer(data, dtype=np.int16)
    volume = np.sqrt(np.mean(audio_data**2))  # Root mean square (RMS) method for volume
    return volume

def hello_world(msg):
    # Non-blocking LED message display for noise detection
    print(msg)
    show_message(device, msg, fill="white", font=proportional(CP437_FONT), scroll_delay=0.1)
    # time.sleep(1)

def display_message(msg):
    # This function runs in a separate thread to display the message without blocking
    threading.Thread(target=hello_world, args=(msg,)).start()

# Track audio direction and point motor
try:
    current_angle = 0  # Track the motor's position
    while True:
        direction = Mic_tuning.direction
        audio_data = stream.read(chunk, exception_on_overflow=False)
        volume = calculate_volume(audio_data)

        # Check if the noise is loud enough to trigger a message
        if volume > THRESHOLD:  # Define a threshold for loud noise
            display_message("Noise Detected!")

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
    stream.stop_stream()
    stream.close()
    p.terminate()
