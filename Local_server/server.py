from flask import Flask, request, redirect, render_template, flash, send_file
import os
from io import BytesIO
from PIL import Image
import numpy as np

app = Flask(__name__)
i = 72
# Directory to store uploaded images
UPLOAD_FOLDER = r"./espimages"
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
            global i
            received_image = Image.open(BytesIO(image_bytes)).convert('RGB')  # Store the image globally
            received_image.save(os.path.join(UPLOAD_FOLDER,f'{i}.jpg'))
            i+=1
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
