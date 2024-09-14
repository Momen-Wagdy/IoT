from flask import Flask, jsonify, request
import threading

class ADC_Node:
    def __init__(self):
        rospy.init_node('adc_node', anonymous=True)
        self.sensors_pub = rospy.Publisher('buzzer_sequence', String, queue_size=10)
        self.flame_pub = rospy.Publisher('flame_water', Bool, queue_size=10)
        self.power_pub = rospy.Publisher('power_off', Bool, queue_size=10)
        self.rate = rospy.Rate(10)
        self.buzzer_sequence = ""
        self.sensor_data = {}  # Dictionary to hold sensor data

        # Start the clock generation in a separate thread
        clock_thread = threading.Thread(target=generate_clock)
        clock_thread.start()

        # Start the Flask server in a separate thread
        flask_thread = threading.Thread(target=self.start_flask_server)
        flask_thread.daemon = True
        flask_thread.start()

    def run(self):
        while not rospy.is_shutdown():
            for ch in range(1, 7):
                self.channel_function(ch)
            self.sensors_pub.publish(self.buzzer_sequence)
            self.buzzer_sequence = ""
            self.rate.sleep(8)
    
    def channel_function(self, ch):
        if (ch == 1):
            self.mq_2(1)
        if (ch == 2):
            self.mq_9(2)
        if (ch == 3):
            self.flame(3)
        if (ch == 5):
            self.current(5)
        if (ch == 6):
            self.voltage(6)

    def mq_2(self, ch):
        result = read_adc(ch)
        vout = (result*5 / 175)
        vin = vout*(7500/(30000+7500))*5
        rospy.loginfo(f"Channel {ch}: {vin}")
        self.sensor_data['mq_2'] = vin
        if (vin > 100):
            self.buzzer_sequence += "B"
        else:
            self.buzzer_sequence += "b"

    def mq_9(self, ch):
        result = read_adc(ch)
        vout = (result*5 / 175)
        vin = vout*(7500/(30000+7500))*5
        rospy.loginfo(f"Channel {ch}: {vin}")
        self.sensor_data['mq_9'] = vin
        if (vin > 100):
            self.buzzer_sequence += "B"
        else:
            self.buzzer_sequence += "b"

    def flame(self, ch):
        result = read_adc(ch)
        vout = (result*5 / 175)
        vin = vout*(7500/(30000+7500))*5
        rospy.loginfo(f"Channel {ch}: {vin}")
        self.sensor_data['flame'] = vin
        if (vin > 100):
            self.buzzer_sequence += "B"
            self.flame_pub.publish(False)
        else:
            self.buzzer_sequence += "b"
            self.flame_pub.publish(True)

    def current(self, ch):
        result = read_adc(ch)
        v = result * (5/255)
        current = (v - 2.5) / 0.066
        rospy.loginfo(f"Channel {ch}: {current}")
        self.sensor_data['current'] = current
        if (current > 100):
            self.power_pub.publish(False)
        else:
            self.power_pub.publish(True)

    def voltage(self, ch):
        result = read_adc(ch)
        vout = (result*5 / 175)
        vin = vout*(7500/(30000+7500))*5
        rospy.loginfo(f"Channel {ch}: {vin}")
        self.sensor_data['voltage'] = vin
        if (vin > 100):
            self.power_pub.publish(False)
        else:
            self.power_pub.publish(True)

    def start_flask_server(self):
        app = Flask(__name__)

        @app.route('/sensor_data', methods=['GET'])
        def get_sensor_data():
            return jsonify(self.sensor_data)

        app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    try:
        node = ADC_Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
