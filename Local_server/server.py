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
    # image = image.resize((640, 640))  # Resize image to 640x640
    # image = np.array(image) / 255.0  # Normalize pixel values to [0, 1]
    # image = np.transpose(image, (2, 0, 1))  # Reorder dimensions for PyTorch (channels, height, width)
    # image = torch.tensor(image, dtype=torch.float32)  # Convert to PyTorch tensor
    # image = image.unsqueeze(0)  # Add batch dimension
    return image

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
            def parse_yolo_output(outputs, confidence_threshold=0.5):
                # Assuming the output tensor is of shape (1, 25200, 8)
                # Remove batch dimension
                outputs = outputs.squeeze(0)  # Now shape is (25200, 8)

                # Convert tensor to numpy array
                outputs = outputs.cpu().numpy()

                # Separate bounding box coordinates and other data
                boxes = outputs[:, :4]  # First 4 columns are bounding box coordinates
                confidences = outputs[:, 4]  # 5th column is object confidence score
                class_scores = outputs[:, 5:]  # Remaining columns are class scores

                # Filter by confidence threshold
                mask = confidences > confidence_threshold
                boxes = boxes[mask]
                confidences = confidences[mask]
                class_scores = class_scores[mask]

                # Find class with the highest score for each detection
                class_ids = class_scores.argmax(axis=1)
                class_confidences = class_scores.max(axis=1)

                # Combine filtered results into a DataFrame
                detections = pd.DataFrame({
                    'xmin': boxes[:, 0],
                    'ymin': boxes[:, 1],
                    'xmax': boxes[:, 2],
                    'ymax': boxes[:, 3],
                    'confidence': confidences,
                    'class_id': class_ids,
                    'class_confidence': class_confidences
                })

                return detections

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
