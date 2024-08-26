from flask import Flask, request, redirect, render_template, flash, send_file
import os
from io import BytesIO
from PIL import Image
import numpy as np
import torch  

app = Flask(__name__)

model = torch.hub.load('./yolov5', 'custom', path='./yolo_model.pt', source='local')
UPLOAD_FOLDER = r"./espimages"
ALLOWED_EXTENSIONS = {'png', 'jpg', 'jpeg'}  
PORT = 19999

if not os.path.exists(UPLOAD_FOLDER): os.makedirs(UPLOAD_FOLDER)

received_image = None

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
    global received_image  
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
            
            received_image = Image.open(BytesIO(image_bytes)).convert('RGB')
            
            image = preprocess_image(image_bytes) 
            
            with torch.no_grad():
                outputs = model(image) 

                detections = outputs.pandas().xyxy[0]  # 'xyxy' format
                
                try:
                    row = detections.iterrows()[0][1]

                    real_width = 5

                    x_min = row['xmin']
                    x_max = row['xmax']
                    mid_point = int((x_min + x_max) / 2)
                    observed_width = x_max - x_min

                    focal_length = 308

                    distance_to_object = int(focal_length * real_width / observed_width)

                    return f"{mid_point}|{distance_to_object}\n"

                except:
                    return "0|0\n"
        else:
            return 'Invalid file type!'

@app.route('/image', methods=['GET'])
def get_image():
    global received_image  
    if received_image is not None:
        img_io = BytesIO()
        received_image.save(img_io, 'JPEG')
        img_io.seek(0)
        return send_file(img_io, mimetype='image/jpeg')
    else:
        return "No image has been received yet."

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=PORT)
