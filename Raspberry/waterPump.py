import rospy
from std_msgs.msg import Bool, String

def flame_callback(data):
    if data.data:  # If True, flame detected
        rospy.loginfo("Flame detected! Activating water pump...")
        # Send a message to activate the second relay (water pump)
        water_pub.publish(True)  # Publish 'on' signal to the water pump
    else:
        rospy.loginfo("No flame detected.")
        # Send a message to deactivate the water pump if no flame
        water_pub.publish(False)  # Publish 'off' signal to the water pump

def flame_listener():
    # Initialize the ROS node
    rospy.init_node('flame_listener', anonymous=True)

    # Publisher to send messages to the 'waterpump_signal' topic
    global pub
    water_pub = rospy.Publisher('waterpump_signal', bool, queue_size=10)

    # Subscribe to the 'flame_detected' topic
    rospy.Subscriber('flame_water', Bool, flame_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        flame_listener()
    except rospy.ROSInterruptException:
        pass
