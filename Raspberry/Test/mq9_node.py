#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import machine
import time

# Set up the ADC pin (e.g., GPIO34)
adc_pin = machine.ADC(machine.Pin(34))
adc_pin.atten(machine.ADC.ATTN_0DB)  # Set attenuation

def read_adc():
    return adc_pin.read()

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('mq9_test_node', anonymous=True)
        
        # Create a publisher
        pub = rospy.Publisher('mq9_test_topic', Float32, queue_size=10)
        
        rospy.loginfo("MQ-9 Test Node Started")
        
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            # Read the ADC value
            adc_value = read_adc()
            # Convert the ADC value to a voltage (example conversion)
            voltage = adc_value * (3.3 / 4095.0)
            # Publish the value
            pub.publish(voltage)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
