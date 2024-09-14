import rospy
import serial
from std_msgs.msg import String, Bool

class ESP32Node:
    def __init__(self):
        rospy.init_node('esp32_node', anonymous=True)
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyS0')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)

        self.buzzer_sub = rospy.Subscriber('buzzer_sequence', String, self.buzzer_control)
        self.power_pub = rospy.Publisher('power', Bool, queue_size=10)

        self.access_granted = False
        self.password = '11111111'

    def buzzer_control(self, msg):
        sequence = msg.data
        for char in sequence:
            self.ser.write(f'{char}\n'.encode('utf-8'))

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.ser.in_waiting:
                response = self.ser.readline().decode('utf-8').strip()
                if response.startswith("password:"):
                    entered_password = response[9:]
                    if (entered_password == self.password):
                        self.access_granted = True
                        self.ser.write(b"V\n")
                        self.power_pub.publish(True)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = ESP32Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
