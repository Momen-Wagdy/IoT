#!/usr/bin/env python

import rospy
import spidev
from std_msgs.msg import Float32

# Set up SPI communication with MCP3008
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0
spi.max_speed_hz = 1000000  # Set the SPI speed

# MCP3008 configuration
def read_adc(channel):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

def flame_sensor_node():
    rospy.init_node('flame_sensor_node', anonymous=True)
    pub = rospy.Publisher('flame_sensor_data', Float32, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        flame_value = read_adc(0)  # Read from channel 0
        rospy.loginfo(f"Flame Sensor Value: {flame_value}")
        pub.publish(flame_value)
        rate.sleep()

if __name__ == '__main__':
    try:
        flame_sensor_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        spi.close()  # Close SPI connection on exit
