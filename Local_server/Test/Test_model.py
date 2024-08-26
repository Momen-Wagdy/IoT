import torch
from PIL import Image
import matplotlib.pyplot as plt
import requests
from io import BytesIO

# Function to download an image from a URL
def download_image(url):
    response = requests.get(url)
    img = Image.open(BytesIO(response.content))
    return img

# Load the model
model = torch.hub.load('./yolov5', 'custom', path='./best.pt', source='local')

# URL of the image
img_url = ''
img = download_image(img_url)

# Perform inference
results = model(img)

# Get results as a pandas DataFrame
detections = results.pandas().xyxy[0]  # 'xyxy' format

# Print out each detection with its coordinates and dimensions
for index, row in detections.iterrows():
    x_min = row['xmin']
    y_min = row['ymin']
    x_max = row['xmax']
    y_max = row['ymax']
    confidence = row['confidence']
    class_id = row['class']
    label = row['name']

    width = x_max - x_min
    height = y_max - y_min

    print(f"Detection {index + 1}:")
    print(f"Label: {label} (Class ID: {class_id})")
    print(f"Confidence: {confidence:.2f}")
    print(f"Coordinates: ({x_min:.2f}, {y_min:.2f}) to ({x_max:.2f}, {y_max:.2f})")
    print(f"Dimensions: Width={width:.2f}, Height={height:.2f}")
    print()

# Optionally display the image with bounding boxes using results.show()
results.show()
