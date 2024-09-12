from flask import Flask, request, redirect, render_template, flash, send_file
import os
from io import BytesIO
from PIL import Image
import pandas as pd
import numpy as np
import cv2
import torch  

app = Flask(__name__)

# Load YOLO model from a local directory
model = torch.hub.load('./yolov5', 'custom', path='./best.pt', source='local',force_reload=True)

# Directory to store uploaded images
UPLOAD_FOLDER = "./espimages"
ALLOWED_EXTENSIONS = {'png', 'jpg', 'jpeg'}  # Allowed image file types
PORT = 19999


# Create the upload folder if it doesn't exist
if not os.path.exists(UPLOAD_FOLDER): 
    os.makedirs(UPLOAD_FOLDER)

# Global variable to store the most recently received image
received_image = None
speed_left, speed_right, brightness = None, None, None
def calculate_brightness(image):
    # Convert image to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Calculate the average brightness
    average_brightness = np.mean(gray_image)
    return average_brightness

def is_brightness_low(image, threshold=50):
    brightness = calculate_brightness(image)
    return brightness < threshold

def fuzzy_inference_system(orientation, distance):
    # Fuzzification of orientation and distance
    right = trapezoidal_mf(orientation, 0, 1, 120, 240)
    center = triangular_mf(orientation, 230, 410)
    left = trapezoidal_mf(orientation, 390, 480, 640, 641)
    close = trapezoidal_mf(distance, 0, 0.1, 15, 25)
    medium = triangular_mf(distance, 20, 55)
    far = trapezoidal_mf(distance, 50, 70, 90, 120)

    # Initialize output possibilities
    speed_left_very_low = 0
    speed_left_low = 0
    speed_left_medium = 0
    speed_left_high = 0
    speed_right_very_low = 0
    speed_right_low = 0
    speed_right_medium = 0
    speed_right_high = 0
    direction_left = 0
    direction_center = 0
    direction_right = 0

    # Apply fuzzy rules
    rule1 = min(right, far)
    speed_left_medium = max(speed_left_medium, rule1)
    speed_right_high = max(speed_right_high, rule1)
    direction_right = max(direction_right, rule1)

    rule2 = min(left, far)
    speed_left_high = max(speed_left_high, rule2)
    speed_right_medium = max(speed_right_medium, rule2)
    direction_left = max(direction_left, rule2)

    rule3 = min(right, medium)
    speed_left_low = max(speed_left_low, rule3)
    speed_right_medium = max(speed_right_medium, rule3)
    direction_right = max(direction_right, rule3)

    rule4 = min(left, medium)
    speed_left_medium = max(speed_left_medium, rule4)
    speed_right_low = max(speed_right_low, rule4)
    direction_left = max(direction_left, rule4)

    rule5 = min(right, close)
    speed_left_very_low = max(speed_left_very_low, rule5)
    speed_right_low = max(speed_right_low, rule5)
    direction_right = max(direction_right, rule5)

    rule6 = min(left, close)
    speed_left_low = max(speed_left_low, rule6)
    speed_right_very_low = max(speed_right_very_low, rule6)
    direction_left = max(direction_left, rule6)

    rule7 = min(center, far)
    speed_left_high = max(speed_left_high, rule7)
    speed_right_high = max(speed_right_high, rule7)
    direction_center = max(direction_center, rule7)

    rule8 = min(center, medium)
    speed_left_medium = max(speed_left_medium, rule8)
    speed_right_medium = max(speed_right_medium, rule8)
    direction_center = max(direction_center, rule8)

    rule9 = min(center, close)
    speed_left_low = max(speed_left_low, rule9)
    speed_right_low = max(speed_right_low, rule9)
    direction_center = max(direction_center, rule9)

    # Defuzzification to get the final crisp output values using center of gravity method
    speed_left = defuzzification(speed_left_low, speed_left_medium, speed_left_high)
    speed_right = defuzzification(speed_right_low, speed_right_medium, speed_right_high)
    direction = defuzzification(direction_left, direction_center, direction_right)

    return speed_left, speed_right

# Symmetric triangular membership function
def triangular_mf(x, a, c, b=None):
    if b is None:
        b = (a + c) / 2
    if x <= a or x >= c:
        return 0
    if x == b:
        return 1
    return (x - a) / (b - a) if x < b else (c - x) / (c - b)

# Asymmetric triangular membership function
def trapezoidal_mf(x, a, b, c, d):
    if x <= a or x >= d:
        return 0.0
    if x >= b and x <= c:
        return 1.0
    if x > a and x < b:
        return (x - a) / (b - a)
    if x > c and x < d:
        return (d - x) / (d - c)
    return 0.0

# Defuzzification using centroid method
def defuzzification(low, med, high):
    # Fuzzy ranges for motor speeds
    low_a, low_b, low_c = 111 - 1e-10, 111, 155
    med_a, med_b, med_c = 140, 167.5, 195
    high_a, high_b, high_c = 180, 255, 255 + 1e-10

    # Solve the inverse for membership functions
    low1 = low * (low_b - low_a) + low_a
    low2 = low_c - low * (low_c - low_b)

    med1 = med * (med_b - med_a) + med_a
    med2 = med_c - med * (med_c - med_b)

    high1 = high * (high_b - high_a) + high_a
    high2 = high_c - high * (high_c - high_b)

    if low + med + high == 0:
        return 0

    # Uses center of gravity weighted mean to calculate wheels speed
    return (low1 * low + low2 * low + med1 * med + med2 * med + high1 * high + high2 * high) / (2 * (low + med + high))

# Function to check if a file is of an allowed type
def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

# Function to preprocess the image before passing it to the model
def preprocess_image(image_bytes):
    image = Image.open(BytesIO(image_bytes)).convert('RGB')  # Convert image to RGB
    return image

@app.route('/arm', methods=["POST", "GET"])
def index():
    global received_image  # Access the global variable received_image
    if request.method == 'GET':
        return render_template('index.html')  # Render the upload form
    elif request.method == 'POST':
        if 'imageFile' not in request.files:
            flash('No file part')  # Flash message if no file part found in the request
            return redirect(request.url)

        file = request.files['imageFile']

        if file.filename == '':
            flash('No selected file')  # Flash message if no file is selected
            return redirect(request.url)

        if file and allowed_file(file.filename):
            image_bytes = file.read()  # Read the image bytes
            
            received_image = Image.open(BytesIO(image_bytes)).convert('RGB')  # Store the image globally
            received_image.save("./i.jpg")
            image = preprocess_image(image_bytes)  # Preprocess the image for model input
            with torch.no_grad():
                outputs = model(image)  # Run inference
                try:
                    detections = outputs.pandas().xyxy[0]  # Convert tensor outputs to DataFrame
                    print(detections)
                    print(outputs)
                except Exception as e:
                    print("No Detections", e)
                    print(outputs)
                    return '0|0|0|0\n'
                try:
                    # Attempt to get the first detected object's bounding box
                    row = detections.iloc[0]
                    print(row)

                    real_width = 5  # Real width of the detected object 

                    x_min = row['xmin']
                    y_min = row['ymin']
                    x_max = row['xmax']
                    y_max = row['ymax']
                    object_class = row['Class']
                    hmid_point = int((x_min + x_max) / 2)  # Calculate the mid-point of the object
                    vmid_point = int((y_min + y_max) / 2)  # Calculate the mid-point of the object
                    observed_width = x_max - x_min  # Calculate the observed width of the object

                    # Not yet calculated, must be calcluated for pi-cam
                    focal_length = 1132.5  # Focal length calculated manually

                    # Calculate the distance to the object based on its observed width
                    distance_to_object = int(focal_length * real_width / observed_width)

                    if distance_to_object < 10: return '-1|-1|-1\n'
                    print(hmid_point,vmid_point,distance_to_object)
                    return f"{hmid_point}|{vmid_point}|{distance_to_object}|{object_class}\n"

                except Exception as e:
                    print("No object",e)
                    return "0|0|0|0\n"  # Return default values if no object is detected
        else:
            return 'Invalid file type!'  # Return error message for invalid file types



@app.route('/', methods=["POST", "GET"])
def index():
    global received_image  # Access the global variable received_image
    if request.method == 'GET':
        return render_template('index.html')  # Render the upload form
    elif request.method == 'POST':
        if 'imageFile' not in request.files:
            flash('No file part')  # Flash message if no file part found in the request
            return redirect(request.url)

        file = request.files['imageFile']

        if file.filename == '':
            flash('No selected file')  # Flash message if no file is selected
            return redirect(request.url)

        if file and allowed_file(file.filename):
            image_bytes = file.read()  # Read the image bytes
            
            received_image = Image.open(BytesIO(image_bytes)).convert('RGB')  # Store the image globally
            received_image.save("./i.jpg")
            image = preprocess_image(image_bytes)  # Preprocess the image for model input

            with torch.no_grad():
                outputs = model(image)  # Run inference
                try:
                    detections = outputs.pandas().xyxy[0]  # Convert tensor outputs to DataFrame
                    print(detections)
                    print(outputs)
                except Exception as e:
                    print("No Detections", e)
                    print(outputs)
                    return '0|0\n'
                try:
                    # Attempt to get the first detected object's bounding box
                    row = detections.iloc[0]
                    print(row)

                    real_width = 5  # Real width of the detected object 

                    x_min = row['xmin']
                    x_max = row['xmax']
                    mid_point = int((x_min + x_max) / 2)  # Calculate the mid-point of the object
                    observed_width = x_max - x_min  # Calculate the observed width of the object

                    focal_length = 1132.5  # Focal length calculated manually

                    # Calculate the distance to the object based on its observed width
                    distance_to_object = int(focal_length * real_width / observed_width)

                    if distance_to_object < 10: return '-1|-1\n'
                    print(mid_point,distance_to_object)
                    global speed_left,speed_right,brightness
                    speed_left,speed_right = fuzzy_inference_system(mid_point,distance_to_object)
                    brightness = abs(int(is_brightness_low(image)) - 1)
                    return f"{int(speed_left)}|{int(speed_right)}|{brightness}\n"

                except Exception as e:
                    print("No object",e)
                    return "0|0\n"  # Return default values if no object is detected
        else:
            return 'Invalid file type!'  # Return error message for invalid file types

@app.route('/image/<time>', methods=['GET'])
def get_image(time):
    global received_image  # Access the global variable received_image
    if received_image is not None:
        img_io = BytesIO()
        received_image.save(img_io, 'JPEG')  # Save the received image to a BytesIO object
        img_io.seek(0)  # Reset file pointer to the beginning
        return send_file(img_io, mimetype='image/jpeg')  # Send the image as a response
    else:
        return "No image has been received yet."  # Return a message if no image is available

@app.route('/suggest/')
def getSuggesting():
    return f"{int(speed_left)}|{int(speed_right)}|{brightness}\n"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=PORT)  # Run the Flask app on the specified port

