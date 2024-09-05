#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class ServoController:
    def _init_(self):
        # Initialize I2C and PCA9685
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50  # Set frequency to 50Hz for servos

        # Create a continuous rotation servo object on channel 0
        self.servo_motor = servo.ContinuousServo(self.pca.channels[0])

        # Initialize ROS node
        rospy.init_node('servo_control_node', anonymous=True)

        # Subscribe to the servo command topic
        rospy.Subscriber('/servo_cmd', Float32, self.servo_callback)

        rospy.loginfo("Servo control node started, waiting for commands...")

    def servo_callback(self, msg):
        # Set servo speed based on the message received (range -1 to 1)
        speed = msg.data

        # Clamp speed to ensure it's within -1 to 1 range
        speed = max(-1.0, min(1.0, speed))

        # Set the speed of the servo
        self.servo_motor.throttle = speed
        rospy.loginfo(f"Set servo speed to: {speed}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = ServoController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:pass
        # Turn off the servo when shutting down
       # controller.pca.deinit()
