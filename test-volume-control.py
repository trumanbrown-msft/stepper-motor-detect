import usb.core
import usb.util
import pyaudio
import numpy as np
from tuning import Tuning
import logging
logging.getLogger('alsaaudio').setLevel(logging.CRITICAL)

# Initialize USB device for ReSpeaker
dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)

if not dev:
    raise ValueError("ReSpeaker 4 Mic Array not found")

Mic_tuning = Tuning(dev)

def compute_rms(audio_data):
    print(audio_data)
    # Ensure data is valid and sanitize NaNs or invalid values
    if audio_data is None or len(audio_data) == 0:
        return 0.0
    audio_data = np.array(audio_data, dtype=np.int16)  # Convert to int16 if needed
    audio_data = np.nan_to_num(audio_data)  # Replace NaNs with 0
    return np.sqrt(np.mean(np.square(audio_data)))  # Calculate RMS

# Initialize PyAudio
p = pyaudio.PyAudio()

# Audio Parameters
RATE = 16000       # Sample rate
CHUNK = 1024       # Smaller chunk for real-time processing
CHANNELS = 6       # ReSpeaker has 6 channels
FORMAT = pyaudio.paInt16
THRESHOLD = 1000   # Lowered threshold to test with

# Find ReSpeaker index dynamically
def find_respeaker():
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        if "ReSpeaker" in info["name"]:
            return info["index"]
    raise ValueError("ReSpeaker not found")

device_index = find_respeaker()

# Open the audio stream
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK,
                input_device_index=device_index)

print("Listening for loud sounds...")

try:
    while True:
        # Read audio data safely
        data = stream.read(CHUNK, exception_on_overflow=False)
        
        # Skip incomplete chunks
        if len(data) < CHUNK:
            print(f"Incomplete chunk received. Expected {CHUNK} bytes, got {len(data)}. Skipping frame.")
            continue  # Skip processing incomplete frames

        # Convert and validate data
        audio_data = np.frombuffer(data, dtype=np.int16)
        audio_data = np.nan_to_num(audio_data)  # Replace NaNs with 0

        # Handle reshaping issues
        if len(audio_data) % CHANNELS != 0:
            print("Invalid chunk size; skipping frame")
            continue

        # Reshape to split channels
        audio_data = audio_data.reshape(-1, CHANNELS)

        # Use channel 0 for processing
        channel_0 = audio_data[:, 0]

        # Calculate RMS safely
        rms = compute_rms(channel_0)

        # Print volume
        print(f"Volume: {rms:.2f}")

        # Check threshold
        if rms > THRESHOLD:
            direction = Mic_tuning.direction  # Get DOA
            print(f"Loud sound detected! Direction: {direction}°")

except KeyboardInterrupt:
    print("Stopping...")

finally:
    stream.stop_stream()
    stream.close()
    p.terminate()
