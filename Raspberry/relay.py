import rospy
from std_msgs.msg import String, Bool
import RPi.GPIO as GPIO
import time

# Define GPIO pins for the relay channels
relay_pins = [4, 17, 27, 10]  # First pin for immediate control, second pin for waterpump signal

def waterpump_callback(msg):
    # Callback function to control the second relay based on waterpump signal
    if msg.data:
        GPIO.output(relay_pins[0], GPIO.LOW)  # Turn second relay (waterpump) on
        rospy.loginfo("Waterpump relay on: GPIO {}".format(relay_pins[0]))
    else:
        GPIO.output(relay_pins[0], GPIO.HIGH)  # Turn second relay (waterpump) off
        rospy.loginfo("Waterpump relay off: GPIO {}".format(relay_pins[0]))

def power_callback(data):
    if data.data:
        GPIO.output(relay_pins[1], GPIO.LOW)
    else:
        GPIO.output(relay_pins[1], GPIO.HIGH)

def relay_control():
    # Set up GPIO mode
    GPIO.setmode(GPIO.BCM)

    # Set up each relay pin as an output
    for pin in relay_pins:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.HIGH)  # Set relay to off initially

    rospy.init_node('relay_control_node', anonymous=True)

    # Publisher to publish the status of the first relay
    relay_pub = rospy.Publisher('relay_status', String, queue_size=10)

    # Subscriber to listen for waterpump signal and control the second relay
    rospy.Subscriber('waterpump_signal', Bool, waterpump_callback)

    relay_sub = rospy.Subscriber('power_off', Bool, power_callback)


    try:
        # Immediately turn on the first relay
        GPIO.output(relay_pins[1], GPIO.LOW)  # Turn first relay on
        relay_pub.publish(f"Relay on: GPIO {relay_pins[1]}")
        rospy.loginfo(f"Relay on: GPIO {relay_pins[1]}")
        
        # Keep the node running and listen for incoming messages
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        GPIO.cleanup()  # Clean up the GPIO on exit

if __name__ == '__main__':
    relay_control()
