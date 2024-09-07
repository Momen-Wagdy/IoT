import board
import busio
import time
import requests
import adafruit_pca9685
import rospy

# Initialize I2C communication using the board's SCL and SDA pins
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize the PCA9685 PWM driver at address 0x40
pca = adafruit_pca9685.PCA9685(i2c, address=0x40)

# Set the PWM frequency to 50 Hz, suitable for controlling servos
pca.frequency = 50

# Define the IP and port for communicating with the AI model
AI_MODEL_IP = "http://192.168.43.89"
AI_MODEL_PORT = "19999"

# Initialize variables to track the previous distance and orientation values
past_distance = -1
past_orientation = -1

# Define a dictionary to map servo names to their respective PCA9685 channels
servos: dict = {"Base": 0, "Elbow": 1, "Spin": 2, "Gripper": 3, "Shoulder": 4}

# Function to stop (lock) the servo by setting it to a neutral position
def lock_servo(channel):
    neutral_duty_cycle = 0x8000  # Neutral duty cycle to stop the servo
    pca.channels[channel].duty_cycle = neutral_duty_cycle  # Set the duty cycle

# Function to move the servo clockwise for a specific duration (pulse)
def move_servo_clockwise(channel, pulse):
    pca.channels[channel].duty_cycle = 0x1200  # Set duty cycle for clockwise movement
    time.sleep(pulse)  # Wait for the specified pulse duration
    lock_servo(channel)  # Stop (lock) the servo after movement

# Function to move the servo counter-clockwise for a specific duration (pulse)
def move_servo_counter_clockwise(channel, pulse):
    pca.channels[channel].duty_cycle = 0x17FF  # Set duty cycle for counter-clockwise movement
    time.sleep(pulse)  # Wait for the specified pulse duration
    lock_servo(channel)  # Stop (lock) the servo after movement

# Function to send a captured image to the AI model via an HTTP POST request
def post_image(image):
    try:
        # Send the image data to the AI model and return the model's response
        response = requests.post(f"{AI_MODEL_IP}:{AI_MODEL_PORT}/", data={"imageFile": image})
        return response.text  # Return the response as text
    except:
        return "-1|-1\n"  # Return a default error response if the request fails

# Function to move the robotic arm based on the response from the AI model
def move_arm(response: str):
    new_distance, new_orientation = response.split("|")  # Parse the response into distance and orientation

    # Check if the response contains invalid values (-1 for both distance and orientation)
    if int(new_distance) == -1 and int(new_orientation) == -1:
        return  # Exit the function if the response is invalid

    # Check if the AI model returned zero for both distance and orientation
    if int(new_distance) == 0 and int(new_orientation) == 0:
        if past_orientation < 280:
            move_servo_clockwise(servos["Base"], 0.15)  # Move the base clockwise
        if past_orientation > 430:
            move_servo_counter_clockwise(servos["Base"], 0.15)  # Move the base counter-clockwise
        else:
            # Add reset code here (optional, if needed for returning servos to default positions)
            pass
    else:
        if int(new_distance) < 5:
            move_servo_clockwise(servos["Gripper"], 0.2)  # Close the gripper if distance is small
        if int(new_orientation) < 280:
            move_servo_counter_clockwise(servos["Base"], 0.2)  # Adjust base counter-clockwise if orientation is small
        if int(new_orientation) > 430:
            move_servo_clockwise(servos['Base'], 0.2)  # Adjust base clockwise if orientation is large
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
            distance = int(new_distance)  # Convert distance to an integer
            # Move the shoulder and elbow to adjust the arm's position
            move_servo_clockwise(servos["Shoulder"], 0.2)
            move_servo_clockwise(servos["Elbow"], 0.2)

        # Update the past distance and orientation values for the next cycle
        global past_distance, past_orientation
        past_orientation, past_distance = int(past_orientation), int(past_distance)

# Placeholder function to capture an image (to be implemented with Raspberry Pi camera)
def capture_image():
    pass  # Needs raspberry pi-cam code to capture an image

# ROS node class to control the robotic arm
class Arm_Node:
    def __init__(self):
        rospy.init_node('Arm_node', anonymous=True)  # Initialize the ROS node
        self.rate = rospy.Rate(0)  # Set the rate of the node's loop (0 indicates no delay)

    def run(self):
        while not rospy.is_shutdown():
            image = capture_image()  # Capture an image (placeholder function)
            response = post_image(image)  # Send the image to the AI model and get the response
            move_arm(response)  # Move the arm based on the AI model's response
            self.rate.sleep()  # Sleep for the defined rate duration

# Main script entry point
if __name__ == '__main__':
    try:
        node = Arm_Node()  # Create an instance of the Arm_Node class
        node.run()  # Run the ROS node loop
    except rospy.ROSInterruptException:
        pass  # Catch and handle ROS interrupt exceptions
