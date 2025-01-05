import RPi.GPIO as GPIO

# Cleanup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.cleanup()  # Resets all GPIO configurations
print("GPIO cleanup complete.")
