#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)
servo_pin = 18  # GPIO pin where the servo is connected
GPIO.setup(servo_pin, GPIO.OUT)

# Set up PWM on the servo pin
pwm = GPIO.PWM(servo_pin, 50)  # 50Hz PWM frequency
pwm.start(7.5)  # Initial position (7.5% duty cycle)

def set_angle(angle):
    duty = angle / 18 + 2.5
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

def callback(data):
    angle = data.data
    if 0 <= angle <= 180:
        set_angle(angle)

def listener():
    rospy.init_node('servo_controller', anonymous=True)
    rospy.Subscriber('servo_angle', Int32, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        pwm.stop()
        GPIO.cleanup()
