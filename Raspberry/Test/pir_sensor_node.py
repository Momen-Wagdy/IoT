#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

# Set up GPIO pin for the PIR sensor
PIR_PIN = 22
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIR_PIN, GPIO.IN)

def pir_sensor_callback(channel):
    # Read the sensor value
    motion_detected = GPIO.input(PIR_PIN)
    print("Motion detected: ", motion_detected)
    # Publish the motion detected status
    pub.publish(motion_detected)

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('pir_sensor_node', anonymous=True)
        
        # Create a publisher
        pub = rospy.Publisher('pir_sensor_topic', Bool, queue_size=10)
        
        # Set up a callback for PIR sensor GPIO input
        GPIO.add_event_detect(PIR_PIN, GPIO.BOTH, callback=pir_sensor_callback)
        
        rospy.loginfo("PIR Sensor Node Started")
        
        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
