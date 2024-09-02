import RPi.GPIO as GPIO
import time

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Define the GPIO pins for the relay channels
relay_pins = [17, 18, 27, 22]  # Change these to the GPIO pins you connected

# Set up each relay pin as an output
for pin in relay_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.HIGH)  # Set relay to off

try:
    while True:
        # Turn on each relay one by one
        for pin in relay_pins:
            GPIO.output(pin, GPIO.LOW)  # Relay on
            time.sleep(1)               # Wait for 1 second

        # Turn off each relay one by one
        for pin in relay_pins:
            GPIO.output(pin, GPIO.HIGH) # Relay off
            time.sleep(1)               # Wait for 1 second

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()  # Clean up the GPIO on exit
