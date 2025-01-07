import usb.core
import usb.util
import time
import RPi.GPIO as GPIO
import numpy as np
import pyaudio
from threading import Thread
from tuning import Tuning
from luma.led_matrix.device import max7219
from luma.core.interface.serial import spi, noop
from luma.core.legacy import show_message
from luma.core.legacy.font import proportional, CP437_FONT


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
THRESHOLD = 30 # Volume threshold for noise detection

# Open the audio stream
stream = p.open(format=format,
                channels=channels,
                rate=rate,
                input=True,
                frames_per_buffer=chunk)

# Function to calculate volume from raw audio data
def calculate_volume(data):
    try:
        # Ensure data is in the expected format
        audio_data = np.frombuffer(data, dtype=np.int16)
        
        if len(audio_data) == 0:
            raise ValueError("Audio data is empty.")

        # Root mean square (RMS) method for volume
        volume = np.sqrt(np.mean(audio_data**2))

        # Prevent invalid values from being returned
        if np.isnan(volume) or volume < 0:
            return 0.0

        return volume
    except Exception as e:
        print(f"Error calculating volume: {e}")
        return 0.0

# Function to show the message on the LED matrix
def led_print(msg):
    print(msg)
    device.clear()  # Clear the screen before showing the message
    show_message(device, msg, fill="white", font=proportional(CP437_FONT), scroll_delay=0.01)  # No scrolling
    # Wait for a short time before clearing the message (e.g., 1 second)
    time.sleep(1)
    device.clear()  # Clear the screen after the message is displayed

# Function to detect loud audio and trigger message display
def audio_processing_thread():
    while True:
        audio_data = stream.read(chunk, exception_on_overflow=False)
        volume = calculate_volume(audio_data)

        # Check if the noise is loud enough to trigger a message
        if volume > THRESHOLD:
            led_print("Noise!")

        time.sleep(0.1)  # Small delay to prevent overloading the CPU

# Track audio direction and point motor
try:
    current_angle = 0  # Track the motor's position

    # Start the audio processing thread
    audio_thread = Thread(target=audio_processing_thread, daemon=True)
    audio_thread.start()

    while True:
        direction = Mic_tuning.direction
        print(f"Audio direction: {direction} degrees")

        # Calculate target angle based on direction
        target_angle = 360 - direction  # Flip direction
        # Normalize the target angle to within 0-360 degrees
        target_angle = target_angle % 360

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

        # Rotate stepper motor (non-blocking)
        step_count = int(abs(angle_diff) / 1.8)  # Steps per revolution (1.8° per step)
        for _ in range(step_count):
            GPIO.output(STEP, GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(STEP, GPIO.LOW)
            time.sleep(0.001)

        # Update current angle
        current_angle = target_angle

        # Small delay to prevent rapid movement
        time.sleep(0.06)

except KeyboardInterrupt:
    print("Exiting program.")
    GPIO.cleanup()
    stream.stop_stream()
    stream.close()
    p.terminate()
