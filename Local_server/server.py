from flask import Flask, request, redirect, render_template, flash, send_file
import os
from io import BytesIO
from PIL import Image
import pandas as pd
import numpy as np
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
                    return f"{mid_point}|{distance_to_object}\n"

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

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=PORT)  # Run the Flask app on the specified port

