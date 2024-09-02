#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

def relay_control():
    # Set up GPIO mode
    GPIO.setmode(GPIO.BCM)

    # Define the GPIO pins for the relay channels
    relay_pins = [17, 18, 27, 22]  # Change these to the GPIO pins you connected

    # Set up each relay pin as an output
    for pin in relay_pins:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.HIGH)  # Set relay to off

    rospy.init_node('relay_control_node', anonymous=True)
    pub = rospy.Publisher('relay_status', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    try:
        while not rospy.is_shutdown():
            # Turn on each relay one by one
            for pin in relay_pins:
                GPIO.output(pin, GPIO.LOW)  # Relay on
                pub.publish(f"Relay on: GPIO {pin}")
                time.sleep(1)               # Wait for 1 second

            # Turn off each relay one by one
            for pin in relay_pins:
                GPIO.output(pin, GPIO.HIGH) # Relay off
                pub.publish(f"Relay off: GPIO {pin}")
                time.sleep(1)               # Wait for 1 second

    except rospy.ROSInterruptException:
        pass

    finally:
        GPIO.cleanup()  # Clean up the GPIO on exit

if __name__ == '__main__':
    relay_control()
