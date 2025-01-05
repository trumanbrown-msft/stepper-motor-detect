import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)

sleepTime = 0.1

# Pins
lightPin = 4
buttonPin = 17

# Setup pins
GPIO.setup(lightPin, GPIO.OUT)
GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.output(lightPin, False)  # Ensure the light is off initially

try:
    while True:
        # Turn on the light when the button is pressed (input is LOW)
        GPIO.output(lightPin, not GPIO.input(buttonPin))
        sleep(sleepTime)
finally:
    GPIO.output(lightPin, False)  # Ensure the light is off before cleanup
    GPIO.cleanup()
