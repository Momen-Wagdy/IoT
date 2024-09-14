import RPi.GPIO as GPIO
import time
import threading
import rospy
from std_msgs.msg import String, Bool

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins
data_pins = [8, 18, 23, 24, 25, 16, 12, 7]  # D0 to D7
addr_pins = [13, 19, 26]  # A, B, C
start_pin = 6  # START
eoc_pin = 21  # EOC
oe_pin = 5  # OE
ale_pin = 22  # ALE
clk_pin = 20  # CLK

# Setup GPIO pins
GPIO.setup(data_pins, GPIO.IN)
GPIO.setup(addr_pins, GPIO.OUT)
GPIO.setup(start_pin, GPIO.OUT)
GPIO.setup(eoc_pin, GPIO.IN)
GPIO.setup(oe_pin, GPIO.OUT)
GPIO.setup(ale_pin, GPIO.OUT)
GPIO.setup(clk_pin, GPIO.OUT)

def generate_clock():
    while True:
        GPIO.output(clk_pin, GPIO.HIGH)
        time.sleep(0.000005)  # Adjust this value for the clock frequency
        GPIO.output(clk_pin, GPIO.LOW)
        time.sleep(0.000005)

def read_adc(channel):
    # Select the channel
    for i in range(3):
        GPIO.output(addr_pins[i], (channel >> i) & 1)
    
    # Latch the address
    GPIO.output(ale_pin, GPIO.HIGH)
    time.sleep(0.001)  # Small delay
    GPIO.output(ale_pin, GPIO.LOW)
    
    # Start conversion
    GPIO.output(start_pin, GPIO.HIGH)
    time.sleep(0.001)
    GPIO.output(start_pin, GPIO.LOW)
    
    rospy.loginfo("Starting conversion.....")
    # Wait for end of conversion
    while GPIO.input(eoc_pin) == GPIO.HIGH:
        time.sleep(0.01)  # Small delay to avoid tight loop
    rospy.loginfo("Conversion done.")
    
    # Enable output
    GPIO.output(oe_pin, GPIO.HIGH)
    
    # Read the 8-bit data
    value = 0
    for i in range(8):
        if GPIO.input(data_pins[i]):
            value |= (1 << i)
    
    # Disable output
    GPIO.output(oe_pin, GPIO.LOW)
    
    return value

class ADC_Node:
    def __init__(self):
        rospy.init_node('adc_node', anonymous=True)
        self.sensors_pub = rospy.Publisher('buzzer_sequence', String, queue_size=10)
        self.flame_pub = rospy.Publisher('flame_water', Bool, queue_size=10)
        self.power_pub = rospy.Publisher('power', Bool, queue_size=10)
        self.fan_pub = rospy.Publusher('fan', Bool, queue_size=10)
        self.rate = rospy.Rate(10)
        self.buzzer_sequence = ""
        self.mq2 = False
        self.mq9 = False

        # Start the clock generation in a separate thread
        clock_thread = threading.Thread(target=generate_clock)
        clock_thread.start()
    
    def update_fan(self):
        if (self.mq2 or self.mq9):
            self.fan_pub.publish(True)
        else:
            self.fan_pub.publish(False)

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
        if (vin > 150):
            self.buzzer_sequence += "B"
            self.mq2 = True
        else:
            self.buzzer_sequence += "b"
            self.mq2 = False
        self.update_Fan()

    def mq_9(self, ch):
        result = read_adc(ch)
        vout = (result*5 / 175)
        vin = vout*(7500/(30000+7500))*5
        rospy.loginfo(f"Channel {ch}: {vin}")
        if (vin > 150):
            self.buzzer_sequence += "B"
            self.mq9 = True
        else:
            self.buzzer_sequence += "b"
            self.mq9 = False
        self.update_fan()

    def flame(self, ch):
        result = read_adc(ch)
        vout = (result*5 / 175)
        vin = vout*(7500/(30000+7500))*5
        rospy.loginfo(f"Channel {ch}: {vin}")
        if (vin > 150):
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
        if (current > 150):
            "self.power_pub.publish(False)"

    def voltage(self, ch):
        result = read_adc(ch)
        vout = (result*5 / 175)
        vin = vout*(7500/(30000+7500))*5
        rospy.loginfo(f"Channel {ch}: {vin}")
        if (vin > 150):
            "self.power_pub.publish(False)"

if __name__ == '__main__':
    try:
        node = ADC_Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
