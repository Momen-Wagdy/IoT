from flask import Flask, request, redirect, render_template, flash
import os
from io import BytesIO
from PIL import Image
import numpy as np
import torch  

app = Flask(__name__)

UPLOAD_FOLDER = r"/home/bruh/espimages"
ALLOWED_EXTENSIONS = {'png', 'jpg', 'jpeg'}  
PORT = 19999

if not os.path.exists(UPLOAD_FOLDER): os.makedirs(UPLOAD_FOLDER)

def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

def preprocess_image(image_bytes):
    image = Image.open(BytesIO(image_bytes)).convert('RGB')
    
    image = image.resize((640, 640))  
    image = np.array(image) / 255.0   
    image = np.transpose(image, (2, 0, 1))  
    image = torch.tensor(image, dtype=torch.float32)  
    image = image.unsqueeze(0) 
    return image

@app.route('/', methods=["POST", "GET"])
def index():
    if request.method == 'GET':
        return render_template('index.html')
    elif request.method == 'POST':
        if 'imageFile' not in request.files:
            flash('No file part')
            return redirect(request.url)

        file = request.files['imageFile']

        if file.filename == '':
            flash('No selected file')
            return redirect(request.url)

        if file and allowed_file(file.filename):
            image_bytes = file.read()
            
            image = preprocess_image(image_bytes)
            # This is the YOLO model, will be added later
            model = Model()  
            
            with torch.no_grad():
                outputs = model(image) 
                
            #
            return 'File uploaded and processed successfully!'
        else:
            return 'Invalid file type!'

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=PORT)
