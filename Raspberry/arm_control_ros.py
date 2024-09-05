import board
import busio
import time
import requests
import adafruit_pca9685
import rospy

i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c, address=0x40)
pca.frequency = 50

AI_MODEL_IP = "http://192.168.43.89"
AI_MODEL_PORT = "19999"

past_distance = -1
past_orientation = -1


servos : dict = {"Base":0, "Elbow":1, "Spin" : 2, "Gripper":3,"Shoulder":4}

# Function to stop (lock) the servo
def lock_servo(channel):
    neutral_duty_cycle = 0x8000 
    pca.channels[channel].duty_cycle = neutral_duty_cycle


def move_servo_clockwise(channel, pulse):
    pca.channels[channel].duty_cycle = 0x1200
    time.sleep(pulse)
    lock_servo(channel)

def move_servo_counter_clockwise(channel, pulse):
    pca.channels[channel].duty_cycle = 0x17FF
    time.sleep(pulse)
    lock_servo(channel)


def post_image(image):
    try:
        response = requests.post(f"{AI_MODEL_IP}:{AI_MODEL_PORT}/", data={"imageFile":image})
        return response.text
    except:
        return "-1|-1\n"


def move_arm(response: str):
    new_distance, new_ortientation = response.split("|")

    if int(new_distance) == -1 and int(new_ortientation) == -1:
        return
    if int(new_distance) == 0 and int(new_ortientation) == 0:
        if past_orientation < 280:
            move_servo_clockwise(servos["Base"], 0.15)
        if past_orientation > 430:
            move_servo_counter_clockwise(servos["Base"], 0.15)
        else:
            # Add reset code here
            pass
    else:
        if int(new_distance) < 5:
            move_servo_clockwise(servos["Gripper"],0.2)
        if int(new_ortientation) < 280:
            move_servo_counter_clockwise(servos["Base"],0.2)
        if int(new_ortientation) > 430:
            move_servo_clockwise(servos['Base'],0.2)

        if int(new_distance) > 5:
            distance = int(new_distance)
            # Check the pulse-time sent, must be configured manually
            move_servo_clockwise(servos["Shoulder"],0.2)
            move_servo_clockwise(servos["Elbow"], 0.2)

        global past_distance, past_orientation
        past_orientation, past_distance = int(past_orientation), int(past_distance)


def capture_image():
    #  Needs raspberry pi-cam code
    pass

class Arm_Node:
    def __init__(self):
        rospy.init_node('Arm_node', anonymous=True)
        self.rate = rospy.Rate(0)

    def run(self):
        while not rospy.is_shutdown():
            image = capture_image()
            response = post_image(image)
            move_arm(response)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Arm_Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
