import rospy
import serial
from std_msgs.msg import String

class ESP32Node:
    def __init__(self):
        rospy.init_node('esp32_node', anonymous=True)
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyS0')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)

        self.buzzer_sub = rospy.Subscriber('buzzer_control', String, self.buzzer_control)
        self.speaker_sub = rospy.Subscriber('speaker_message', String, self.speaker_message)

        self.access_granted = False
        self.password = '11111111'

    def buzzer_control(self, msg):
        state = msg.data
        if (state == 'ON'):
            self.ser.write(b'B\n')
        elif (state == 'OFF'):
            self.ser.write(b'b\n')

    def speaker_message(self, msg):
        message = msg.data
        self.ser.write(f'S\n{message}\n'.encode())

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
            rate.sleep()

if __name__ == '__main__':
    try:
        node = ESP32Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
